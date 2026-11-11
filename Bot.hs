-- A robot on the Clunkyard core's Haskell binding. The core owns tracking,
-- geometry and protocol handling; this strategy chooses the tactics.
-- Clunkyard robot API 2.0.0: one core under every language, the same
-- answers everywhere. Facts (a call shows its arguments):
-- match: mode ruleset control robots is_3d tick_rate max_ticks
--   robot_id(slot)
-- clock: tick seconds remaining_ticks
-- arena: size gravity terrain_kind ground_height(x y)
-- me: health position velocity heading pitch roll angular_velocity
--   attitude grounded altitude applied
-- battery: charge capacity regen sustainable_drive compute
--   drive_cost(fraction)
-- drive: hover_lift
-- gun: aim_now cooldown_ticks spread max_power damage_scale
--   can_fire(power reserve) shot_speed(power) shot_cost(power)
--   reload_ticks(power) shot_damage(power) power_for_speed(speed)
--   power_for_flight_time(range seconds) lead(contact power)
--   lead_point(position velocity power)
-- scanner: last proximity_range reach(arc) arc_for_reach(range)
-- sensors: contacts nearest cues contact(slot) position_of(contact)
-- tracker: cue nearest_bearing_rate velocity(slot) position(slot)
--   last_seen(slot)
-- nav: zone zone.margin(point) zone.contains(point margin)
--   zone.distance_from(point) storm storm.margin(point)
--   storm.contains(point margin) storm.distance_from(point) hill
--   hill.margin(point) hill.contains(point margin)
--   hill.distance_from(point) pickups nearest_pickup(kind)
--   field_distance(a b)
-- score: me best armour
-- angles: normalize(degrees) angle_diff(target current)
--   bearing_to(from to) elevation_to(from to) direction(heading pitch)
--   distance(a b) clamp(value low high)
-- raw: number(path) text(path) has(path) flag(path) count(path) line()
-- Controls: drive.set(throttle turn pitch_turn)
--   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
--   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
--   tracker.configure(smoothing forget_ticks cue_ticks)
--   status.set(text value) raw.extra(json)
-- Every control argument is optional; a second call keeps what the first
-- set. Absent facts are your language's none. Names are snake_case here;
-- your language cases them its own way. Angles are degrees from the +x
-- axis; pitch is positive up.
-- Manual: https://clunkyard.com/manual#sdk
{-# LANGUAGE BangPatterns #-}
module Main where
import Clunkyard
import Foreign
import Foreign.C
import Data.Bits
import Data.Word
import Data.IORef
import Control.Monad

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
-- The arena's calibration kernel, xorshift-sqrt v1; safe to delete.
calibrationWork :: Int -> Int -> CDouble
calibrationWork work tick = fromIntegral ((floor total + fromIntegral (shiftR final 40)) `mod` (1000000 :: Integer))
  where
    (final,total) = go work (88172645463325252 `xor` fromIntegral tick) 0
    go :: Int -> Word64 -> Double -> (Word64,Double)
    go 0 x total = (x,total)
    go n !x !total = let a=x `xor` shiftL x 13; b=a `xor` shiftR a 7; c=b `xor` shiftL b 17 in go (n-1) c (total+sqrt (fromIntegral (shiftR c 11)))
start free robot = do
  is3d <- query robot (cyMatchIs3d robot)
  gravity <- query robot (cyArenaGravity robot)
  writeIORef free (is3d /= 0 && vec3Z gravity == 0)
turnBot free robot = do
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
      scan robot (contactBearing t) 20 (contactElevation t)
      power <- query robot (cyGunPowerForFlightTime robot (contactRange t) 1)
      solution <- with t $ \ptr -> optional robot (cyGunLead robot ptr power)
      let (aimHeading,aimPitch) = maybe (contactBearing t,contactElevation t) (\s -> (solutionHeading s,solutionPitch s)) solution
      cyGunAimHeading robot aimHeading >>= check robot
      cyGunAimPitch robot aimPitch >>= check robot
      now <- query robot (cyGunAimNow robot)
      delta <- query robot (cyAnglesAngleDiff aimHeading (directionHeading now))
      canFire <- query robot (cyGunCanFire robot power 10)
      when (abs delta < 3 && abs (aimPitch-directionPitch now) < 3 && canFire /= 0) $ do
        cyGunFire robot >>= check robot
        cyGunPower robot power >>= check robot
      showStatus robot "TRACK" (Just (contactRange t))
    Nothing -> do
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
          scan robot sweep 40 (if freeFlight then fromIntegral ((tick `div` 10) `mod` 3-1)*35 else 0)
          showStatus robot "SEARCH" Nothing
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
  -- Only the arena's rate calibration sends this; safe to delete.
  work <- withText "state.calibration.work" $ \path count -> optional robot (cyRawNumber robot path count)
  forM_ work $ \n -> when (n >= 1 && n == fromInteger (floor n)) $ showStatus robot "WORK" (Just (calibrationWork (floor n) (fromIntegral tick)))
main = do
  free <- newIORef False
  run (turnBot free) (start free)
