#ifndef LAZYSERVO_HH
#define LAZYSERVO_HH

//--- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
//--- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
// FAN / BALL-VALVE-SERVO / ETC - VENTILATION / TEMPERATURE CONTROL
//--- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -


// FACTS:
// - A normal servo expects a pulse renewal every 20ms (20,000 us)
// - speed (s/60°), reports from 0.08s - 0.18s, 0.15s seems reasonable mid
//    - the 9g's is reported as 0.12s/4.8V with _no load_
//    - err on the slow side for repeated pulses before sleeping

#include <Servo.h>
#include <UnitRangeControl.h>
#include <OnyxTypeAliases.h>


#define def_state static const ChronoState


template <PinNumber PIN, typename RT = F32>
class LazyServo : public UnitRangeControl, public ChronoTrigger {
   RT       move_lazyness_thresh;

   // how often to update stats
   //    - best as slow as possible wrt doze-timeout etc. and instead `set` should actively update
   //    - rebuild as mentioned - and then this can be removed and timeout_before_dozing used (and "infinite" if not used [but then: why use LazyServo? ;-) ]
   TimeSpan check_interval = 100UL * 1000;
   /* U16   full_rotate_duration; */
   U16      timeout_before_dozing;
   /* U16      min_rotate_duration; */
   RT       min_usecs;
   RT       max_usecs;

   RT       prev_adjusted_value = RT(-1.0);
   RT       target_value = RT(0.5);  // Center it as initial target_position_micros (value in unit percent)

   Servo    servo;   // create servo object to control a servo

  public:
   def_state   MonitorWhileAwake    = 1;
   def_state   MonitorWhileAsleep   = 2;
   def_state   ServoSleep           = 3;
   def_state   ServoWakeup          = 4;

   LazyServo(
      RT          move_lazyness_thresh_ =    0.001,  // this should just adjust the stay-in-sleep time: (n >= x) => immediate, (0 < n < x)  =>  use stay-in-sleep-timeout (re-use `timeout_before_dozing_`)

      TimeSpan    check_interval_ =          100UL * 1000,  // how often to update stats
      /* TimeSpan    full_rotate_duration_ = 1500, */
      U16         timeout_before_dozing_ =   2000UL * 1000,
      /* TimeSpan    min_rotate_duration_ =  20, */
      U16         min_usecs_ =               500, // 544,
      U16         max_usecs_ =               2500, // 2400,
      RT          initial_position_ =        RT(0.5)
   ) :
      move_lazyness_thresh(move_lazyness_thresh_),
      check_interval(check_interval_),
      /* full_rotate_duration(full_rotate_duration_), */
      timeout_before_dozing(timeout_before_dozing_),
      /* min_rotate_duration(min_rotate_duration_), */
      target_value(initial_position_)
   {
      set_servo_limits(min_usecs_, max_usecs_);
      go_next(MonitorWhileAsleep);
   }

   inline
   fn set_servo_limits(U16 min_usecs_, U16 max_usecs_) -> Void {
      min_usecs = min_usecs_;
      max_usecs = max_usecs_;
   }

   inline
   fn set(RT value_) -> Void {
      target_value = value_;
      /* adjust_if(); */
   }

   inline
   fn set_now(RT value_) -> Void {
      target_value = value_;
      update();
   }

   inline
   fn get() -> RT {
      return target_value;
   }

   inline
   fn is_ready() -> Bool { return true; }

   fn update() -> Void {
      /* DBG("Servo#update("); */
      /* DBG(where_to_go()); */
      /* DBG(")"); */
      /* DBGn(); */

      switch (where_to_go()) {
      case MonitorWhileAsleep:
         /* DBG("MonitorWhileAsleep"); */
         /* DBGn(); */

         if (breach_lazyness_threshold()) { // this could simply be checked WHEN SETTING NEW TARGET
            cancel_scheduled_state();
            go_next(ServoWakeup);
         } else {
            /* sleep(check_interval); */
            go_after_sleep(MonitorWhileAsleep, check_interval);
         }
      break;

      case MonitorWhileAwake:
         /* DBG("MonitorWhileAwake"); */
         /* DBGn(); */

         if (breach_lazyness_threshold()) { // this could simply be checked WHEN SETTING NEW TARGET
            adjust_servo(); // go_next(Adjust);
            go_after(MonitorWhileAwake, check_interval, true);
            /* sleep(check_interval); */
            // on_timeout_go(ServoSleep, timeout_before_dozing);  // <-- BETTER WAY OF DOING THIS!
         } else {
            go_after(ServoSleep, timeout_before_dozing, false);
         }
      break;

      case ServoWakeup:
         DBG("ServoWakeup");
         DBGn();

         servo.attach(PIN);
         adjust_servo();
         go_next(MonitorWhileAwake);
      break;

      case ServoSleep:
         DBG("ServoSleep");
         DBGn();

         servo.detach();
         go_next(MonitorWhileAsleep);
      break;
      }
   }

   fn log() -> Void {}

  private:
   fn adjust_servo() -> Void {
      DBG("Servo: Adjust to ");
      DBG(target_value);

      // Serial.print("writes target_position_micros ");
      // Serial.print(target_position_micros);
      // Serial.print("to servo.");
      // Serial.println();

      U16 target_position_micros = lround(min_usecs + ((max_usecs - min_usecs) * target_value));

      DBG(" (");
      DBG(target_position_micros);
      DBG(")");
      DBGn();

      servo.writeMicroseconds(target_position_micros);

      prev_adjusted_value = target_value;
   }

   inline
   fn breach_lazyness_threshold() -> Bool {
      return (abs(target_value - prev_adjusted_value) > move_lazyness_thresh);
   }
};

#endif
