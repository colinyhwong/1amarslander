// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <fstream>

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
  double altitude = position.abs() - MARS_RADIUS;
  double Pout = -KP * (0.5+KH*altitude+velocity*position/position.abs());
  double bias = (UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY) / (1.5*(UNLOADED_LANDER_MASS + FUEL_DENSITY * FUEL_CAPACITY));
  if (Pout > 1 - bias) {
	  throttle = 1;
  }
  else if (Pout < -bias){
	  throttle = 0;
  }
  else {
	  throttle = bias + Pout;
  };
  //cout << "Throttle :" << throttle << " \n";
  //cout << "Pout :" << Pout << " \n\n";
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // Update mass
  double mass = UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY; //currently ignoring fuel
  
  // Drag force
  vector3d drag_force = -0.5*PI*LANDER_SIZE * LANDER_SIZE*DRAG_COEF_LANDER*atmospheric_density(position)*velocity.abs()*velocity;
  if (parachute_status == DEPLOYED) {
	  drag_force += -10*LANDER_SIZE * LANDER_SIZE*DRAG_COEF_CHUTE*atmospheric_density(position)*velocity.abs()*velocity;
  }

  //Thrust force
  vector3d thrust_force = thrust_wrt_world();

  //Gravitation force
  vector3d grav_force = -position*mass* MARS_MASS * GRAVITY / (position.abs()*position.abs()*position.abs());

  //numerical intergration for position
  static vector3d previous_position;
  if (simulation_time == 0.0) {
	  previous_position = position - delta_t * velocity;
  }
  vector3d acceleration = (grav_force+drag_force+thrust_force)/mass;
  vector3d updated_position = 2 * position - previous_position + delta_t * delta_t*acceleration;
  velocity = (updated_position - previous_position) / (2*delta_t);
  previous_position = position;
  position = updated_position;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();


  //Writeout
  ofstream fout;
  string filename = "./traj/" + to_string(KP) + "_" + to_string(KH) + "_" + to_string(KT) + ".txt";
  if (simulation_time == 0.0) {
	  fout.open(filename);
  }
  else {
	  fout.open(filename, ios::app);
  }
  fout << simulation_time <<" "<< velocity.abs() <<" "<< position.abs() - MARS_RADIUS <<"\n";
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "Aerostationary Orbit";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
	// Aerostationary Orbit
	position = vector3d(cbrt(GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY/(4*PI*PI)),0.0, 0.0);
	velocity = vector3d(0.0,cbrt(2*PI*GRAVITY*MARS_MASS/MARS_DAY), 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = true;
	autopilot_enabled = false;
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
