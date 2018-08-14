A servo-controller for cases where there's no load to mention on the servo,
where LazyServo will simply turn the servo off when resting, and turn it 
back on and move if the position changes. This saves batteries and more
importantly, the servo, since they often have some jitter.

