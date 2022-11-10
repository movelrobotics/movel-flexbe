# movel_flexbe
This repo contains FlexBE states for executing Seirios RNS tasks, example behaviours, and some custom flexbe launch files.

## Installation
1. Clone this repo inside the `src` folder in a ROS workspace together with your other FlexBE states and behaviours.

```bash
$ cd your_flexbe_workspace/src
$ git clone https://gitlab.com/movelai_public/movel_flexbe.git
```
2. As a dependency for this stack, you also need to have [movel_seirios_msgs](https://gitlab.com/movelai_public/movel_seirios_msgs) pacakge installed.
3. Build the workspace.

## Usage
We provide two launchfiles to launch FlexBE behaviour engine: with and without the FlexBE app GUI. Since we are using additional config files for our states, it is recommended to use our launchfiles instead of the default ones when working with our FlexBE states.

### Without UI
The default way to use FlexBE with Seirios RNS software is running FlexBE onboard behaviour engine headless. To do that, use the flexbe_headless launch.
```bash
$ roslaunch movel_flexbe flexbe_headless.launch
```
This will launch FlexBE with an action server that interfaces with Seirios RNS.

### With UI
When creating or editing a behaviour, it is practical to use the built-in FlexBE GUI. For this, use the flexbe_full_ui launch.
```bash
$ roslaunch movel_flexbe flexbe_full_ui.launch
```
This will launch FlexBE with the usual GUI and the states will still be able to interact with Seirios RNS.
> In order to use the GUI, you must also have the FlexBE app package compiled. See [here](https://github.com/FlexBE/flexbe_app).

## Configuration File
In the movel_flexbe package is a `config` folder that contains `seirios_api.yaml`. This yaml file contains information about Seirios RNS API address and user credentials that will be used by some `movel_flexbe` states when communicating with the Seirios server.
```yaml
address: "localhost:8000"   # the API address
user:                       # Seirios RNS username for API access
  username: "movel_flexbe"
  password: "movel_flexbe"
```

## Available States
Details of available states are provided [here](movel_flexbe_states).