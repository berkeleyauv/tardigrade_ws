## Summary

- 

## Testing

- [ ] `./build.sh`
- [ ] `colcon test --packages-select tardigrade_interfaces tardigrade_state_estimation tardigrade_esp tardigrade_teleop tardigrade_bringup tardigrade_mission tardigrade_sim`
- [ ] Hardware/Jetson test, if needed:

## Risk Checklist

- [ ] I called out any changes that affect firmware, serial protocols, or thruster mapping.
- [ ] I called out any changes that affect state estimation topics, frames, or covariance assumptions.
- [ ] I updated docs or runbooks for changed operator-facing behavior.
- [ ] This can be reviewed without access to the robot, or I documented what requires robot hardware.
