#Milestone 1
MY approach is to provide different magnitudes of velociy to both the joints to so that there is relative motion between resulting in a circular motion. The radius of the circle can be adjusted by accordingly setting the velocity magnitudes. Larger the difference in magnitude, tighter is the circle.

Circle 1:
	r_joint = sim.getObject("/crn_bot/joint_r")
	l_joint = sim.getObject("/crn_bot/joint_l")
	sim.setJointTargetVelocity(int(r_joint), 10)
	sim.setJointTargetVelocity(int(l_joint), -7)

 Circle 2:
  r_joint = sim.getObject("/crn_bot/joint_r")
	l_joint = sim.getObject("/crn_bot/joint_l")
	sim.setJointTargetVelocity(int(r_joint), 3)
	sim.setJointTargetVelocity(int(l_joint), -10)

 Circle 3:
  r_joint = sim.getObject("/crn_bot/joint_r")
	l_joint = sim.getObject("/crn_bot/joint_l")
	sim.setJointTargetVelocity(int(r_joint), 2.5)
	sim.setJointTargetVelocity(int(l_joint), -4)
