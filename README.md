# ND AgJunction Webots

In this project, AgJunction requested “medium fidelity solid models” of the two tractors in their inventory (Kubota M7 and McCormick MTX120) to use for simulation, as an improvement to the previously-used rigid body model with perfect steering behavior. The new models were to account for suspension and steering factors not previously considered, and the visual renderings were to be made more reminiscent of the tractors in subject. 

This project is collaborated with ***SUSTech***, ***ND*** and ***AgJunction Inc***. 

## Quick Start
1. Check out this repository and download the source code

    `git clone git@github.com:silvery107/ND-agjunction-webots.git`
2. Open `worlds/AgJunctionProject.wbt`
3. Check `tractor_controller`
   
- This work is implemented under the version of `R2020a`.

## [Model Description](./docs/model_description.md)

<img src=docs/images/tractor_proto.png width=400>

- [Model Description](./docs/model_description.md#model-description)
  - [Tractor PROTO](./docs/model_description.md#tractor-proto)
    - [PROTO Field Summary](./docs/model_description.md#proto-field-summary)
  - [Engine Output](./docs/model_description.md#engine-output)
  - [Robot Window](./docs/model_description.md#robot-window)
  - [Sensor Slot](./docs/model_description.md#sensor-slot)
    - [Accelerometer](./docs/model_description.md#accelerometer)
      - [Description](./docs/model_description.md#description)
      - [Field Summary](./docs/model_description.md#field-summary)
    - [GPS](./docs/model_description.md#gps)
      - [Description](./docs/model_description.md#description-1)
      - [Field Summary](./docs/model_description.md#field-summary-1)
    - [Gyro](./docs/model_description.md#gyro)
      - [Description](./docs/model_description.md#description-2)
      - [Field Summary](./docs/model_description.md#field-summary-2)
    - [InertialUnit](./docs/model_description.md#inertialunit)
      - [Description](./docs/model_description.md#description-3)
      - [Field Summary](./docs/model_description.md#field-summary-3)


## [Controller Description](./docs/controller_description.md)

<img src=docs/images/overview_tab.png width=400>

- [Controller Description](./docs/controller_description.md#controller-description)
  - [Driver Library Functions](./docs/controller_description.md#driver-library-functions)
  - [Keyboard](./docs/controller_description.md#keyboard)
  - [InertialUnit](./docs/controller_description.md#inertialunit)
  - [Accelerometer](./docs/controller_description.md#accelerometer)
  - [GPS](./docs/controller_description.md#gps)
  - [Robot](./docs/controller_description.md#robot)