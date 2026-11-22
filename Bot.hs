-- Narrow the scan around a contact and lead shots from recent sightings.
-- Clunkyard guide: https://clunkyard.com/sdk/haskell
module Main where
import Clunkyard
import Foreign
import Foreign.C
import Data.IORef
import Control.Monad

searchArc, holdWidth, minArc :: CDouble
searchArc = 40  -- degrees: sweeps the circle in half a second
holdWidth = 60  -- units across at the target: it stays in the beam from tick to tick
minArc = 6      -- degrees: the narrowest the beam holds
window, lookEvery, reacquire :: CInt
window = 12     -- ticks of sightings the aim averages
lookEvery = 40  -- ticks between looks around the target, for a nearer robot
reacquire = 10  -- ticks to look again where a lost target was
sightings :: Int
sightings = 8   -- narrow sightings in the window before a shot

steer robot target current scale = do
  delta <- query robot (cyAnglesAngleDiff target current)
  query robot (cyAnglesClamp (delta / scale) (-1) 1)
drive robot throttle turn pitch = do
  cyDriveThrottle robot throttle >>= check robot
  cyDriveTurn robot turn >>= check robot
  cyDrivePitchTurn robot pitch >>= check robot
scan robot heading arc pitch = do
  cyScannerHeading robot heading >>= check robot
  cyScannerArc robot arc >>= check robot
  cyScannerPitch robot pitch >>= check robot
showStatus robot text value = do
  withText text (cyStatusText robot) >>= check robot
  forM_ value $ \number -> cyStatusValue robot number >>= check robot
-- Whether this tick's passive contacts include a shot heard.
heardShot robot = allocaArray 64 $ \buffer -> alloca $ \count -> do
  cySensorsCues robot buffer 64 count >>= check robot
  n <- peek count
  cues <- peekArray (min 64 (fromIntegral n)) buffer
  pure (any ((== cySourceMuzzle) . contactSource) cues)
-- A straight line through the narrowest of the last window ticks' sightings:
-- where the robot is now and how fast it moves, and how many sightings it
-- rests on. Each sighting is off by its bearing error; the line averages that out.
fitLine :: CInt -> CInt -> [CySighting] -> Maybe (CyVec3, CyVec3, Int)
fitLine tick rate history
  | length recent < 2 || length seen < 2 = Nothing
  | otherwise = Just (CyVec3 px py pz, CyVec3 vx vy vz, length seen)
  where
    recent = filter ((> tick - window) . sightingTick) history
    best = minimum (map sightingBearingError recent)
    seen = filter ((<= 1.5 * best) . sightingBearingError) recent -- the narrow ones
    n = fromIntegral (length seen) :: Double
    ticks = map (fromIntegral . sightingTick) seen :: [Double]
    t0 = sum ticks / n
    spread = sum [(t - t0) ^ (2 :: Int) | t <- ticks]
    -- One axis: the line's position now and its slope per tick.
    axis along = (realToFrac (mean + slope * (fromIntegral tick - t0)), realToFrac (slope * fromIntegral rate))
      where
        points = map (realToFrac . along . sightingPosition) seen :: [Double]
        mean = sum points / n
        slope = sum (zipWith (\t p -> (t - t0) * (p - mean)) ticks points) / spread
    ((px, vx), (py, vy), (pz, vz)) = (axis vec3X, axis vec3Y, axis vec3Z)
start free robot = do
  is3d <- query robot (cyMatchIs3d robot)
  gravity <- query robot (cyArenaGravity robot)
  writeIORef free (is3d /= 0 && vec3Z gravity == 0)
turnBot free dodging slot robot = do
  freeFlight <- readIORef free
  position <- query robot (cyMePosition robot)
  heading <- query robot (cyMeHeading robot)
  pitch <- query robot (cyMePitch robot)
  tick <- query robot (cyClockTick robot)
  cyDriveBrake robot 0.2 >>= check robot
  target <- optional robot (cySensorsNearest robot)
  case target of
    Just t -> do
      let throttle = if contactRange t > 350 then 0.7 else if contactRange t < 200 then -0.4 else 0.2
      yaw <- steer robot (contactBearing t) heading 45
      climb <- steer robot (contactElevation t) pitch 45
      drive robot throttle yaw climb
      writeIORef slot (contactSlot t)
      -- Halve the beam each tick until it is holdWidth across at the target:
      -- narrow enough for a precise bearing, wide enough to keep it in.
      lastArc <- scanArc <$> query robot (cyScannerLast robot)
      let hold = max minArc (2 * atan2 (holdWidth/2) (contactRange t) * 180 / pi)
          -- In a crowd a narrow beam sees one robot. Every lookEvery ticks,
          -- widen it for a tick: a nearer robot it finds becomes the target.
          arc = if tick `mod` lookEvery == 0 then searchArc else max hold (min searchArc (lastArc/2))
      scan robot (contactBearing t) arc (contactElevation t)
      power <- query robot (cyGunPowerForFlightTime robot (contactRange t) 1)
      -- An exact contact: cyGunLead is the intercept if the target holds course.
      -- A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
      (solution,ready) <- if contactBearingError t > 0
        then do
          rate <- query robot (cyMatchTickRate robot)
          recorded <- allocaArray 32 $ \buffer -> alloca $ \count -> do
            cyTrackerHistory robot (contactSlot t) buffer 32 count >>= check robot
            n <- peek count
            peekArray (min 32 (fromIntegral n)) buffer
          case fitLine tick rate recorded of
            Nothing -> pure (Nothing,False)
            Just (spot,velocity,count) -> do
              lead <- with spot $ \point -> with velocity $ \speed -> optional robot (cyGunLeadPoint robot point speed power)
              pure (lead,count >= sightings)
        else do
          lead <- with t $ \ptr -> optional robot (cyGunLead robot ptr power)
          pure (lead,True)
      let (aimHeading,aimPitch) = maybe (contactBearing t,contactElevation t) (\s -> (solutionHeading s,solutionPitch s)) solution
      cyGunAimHeading robot aimHeading >>= check robot
      cyGunAimPitch robot aimPitch >>= check robot
      now <- query robot (cyGunAimNow robot)
      delta <- query robot (cyAnglesAngleDiff aimHeading (directionHeading now))
      canFire <- query robot (cyGunCanFire robot power 10)
      when (ready && abs delta < 3 && abs (aimPitch-directionPitch now) < 3 && canFire /= 0) $ do
        cyGunFire robot >>= check robot
        cyGunPower robot power >>= check robot
      showStatus robot (if ready then "TRACK" else "NARROW") (Just (contactRange t))
    Nothing -> do
      seen <- readIORef slot
      lastSeen <- if seen == 0 then pure Nothing else optional robot (cyTrackerLastSeen robot seen)
      case lastSeen of
        Just at | tick - at <= reacquire -> do
          -- Lost it: look again where it was last seen, widening the beam, before searching.
          remembered <- optional robot (cyTrackerPosition robot seen)
          lastArc <- scanArc <$> query robot (cyScannerLast robot)
          with position $ \pos -> with (maybe position id remembered) $ \point -> do
            bearing <- query robot (cyAnglesBearingTo pos point)
            elevation <- if freeFlight then query robot (cyAnglesElevationTo pos point) else pure 0
            yaw <- steer robot bearing heading 45
            climb <- steer robot elevation pitch 45
            drive robot 0.5 yaw climb
            scan robot bearing (min searchArc (lastArc*2)) elevation
            showStatus robot "REACQUIRE" Nothing
        _ -> do
          heard <- optional robot (cyTrackerCue robot)
          case heard of
            Just cue -> with position $ \pos -> with (cuePosition cue) $ \point -> do
              bearing <- query robot (cyAnglesBearingTo pos point)
              distance <- query robot (cyAnglesDistance pos point)
              elevation <- if freeFlight then query robot (cyAnglesElevationTo pos point) else pure 0
              let spread=min (cueBearingError cue) 30; across=fromIntegral (tick `mod` 5-2)/2*spread; up=if freeFlight then fromIntegral ((tick `div` 5) `mod` 3-1)*spread/2 else 0
              yaw <- steer robot bearing heading 45
              climb <- steer robot elevation pitch 45
              drive robot 0.7 yaw climb
              arc <- query robot (cyScannerArcForReach robot (distance+100))
              scan robot (bearing+across) arc (elevation+up)
              showStatus robot "CUE" (Just distance)
            Nothing -> do
              lastScan <- query robot (cyScannerLast robot)
              sweep <- query robot (cyAnglesNormalize (scanHeading lastScan+36))
              climb <- steer robot 0 pitch 45
              drive robot 0.5 0.33 climb
              scan robot sweep searchArc (if freeFlight then fromIntegral ((tick `div` 10) `mod` 3-1)*35 else 0)
              showStatus robot "SEARCH" Nothing
  -- A simple dodge: vary it, or predict the shooter.
  (taken,ends) <- readIORef dodging
  shot <- if tick >= ends then heardShot robot else pure False
  when shot $ writeIORef dodging (taken+1, tick+10)
  (dodges,dodgeEnd) <- readIORef dodging
  when (tick < dodgeEnd) $ do
    -- Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
    let left = odd (dodges :: Int)
    control <- query robot (cyMatchControl robot)
    if control == cyControlForces
      then cyDriveThrottle robot 0.5 >>= check robot >> cyDriveStrafe robot (if left then -1 else 1) >>= check robot
      else cyDriveThrottle robot 1 >>= check robot >> cyDriveTurn robot (if left then 1 else -1) >>= check robot
    showStatus robot (if left then "dodge:left" else "dodge:right") (Just (fromIntegral dodges))
  charge <- query robot (cyBatteryCharge robot)
  capacity <- query robot (cyBatteryCapacity robot)
  when (charge < capacity/4) $ do
    throttle <- optional robot (cyDriveGetThrottle robot)
    forM_ throttle $ \value -> cyDriveThrottle robot (value*0.5) >>= check robot
  with position $ \pos -> do
    storm <- optional robot (cyNavStorm robot)
    forM_ storm $ \s -> do
      margin <- optional robot (cyNavStormMargin robot pos)
      when (maybe False (> (-60)) margin) $ with (stormCenter s) $ \center -> do
        away <- query robot (cyAnglesBearingTo center pos)
        yaw <- steer robot away heading 30
        cyDriveThrottle robot 0.8 >>= check robot
        cyDriveTurn robot yaw >>= check robot
    zone <- optional robot (cyNavZone robot)
    forM_ zone $ \z -> do
      margin <- optional robot (cyNavZoneMargin robot pos)
      when (maybe False (<60) margin) $ with (zoneCenter z) $ \center -> do
        bearing <- query robot (cyAnglesBearingTo pos center)
        elevation <- if zoneShape z == cyShapeSphere then query robot (cyAnglesElevationTo pos center) else pure 0
        yaw <- steer robot bearing heading 30
        climb <- query robot (cyAnglesClamp ((elevation-pitch)/30) (-1) 1)
        drive robot 0.8 yaw climb
main = do
  free <- newIORef False
  dodging <- newIORef (0, 0) -- dodges taken, and the tick the current one ends
  slot <- newIORef 0 -- the last target's slot, for looking again where it was lost
  run (turnBot free dodging slot) (start free)
