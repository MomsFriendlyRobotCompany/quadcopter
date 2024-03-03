# Flight Controller

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Idle: boot
    Idle --> [*]: shutdown
    Idle --> Armed: arm
    Armed --> Idle: unarm

    state Armed {
        [*] --> Land
        Land --> Fly: takeoff
        Fly --> Land: land
        Land --> [*]: unarm
    }

    state Idle {
        state if_cal <<choice>>
        [*] --> if_cal
        if_cal --> calibrate: Calibrated False
        if_cal --> ready: Calibrated True
        calibrate --> ready: complete
        ready --> calibrate: calibration msg
        ready --> safe: shutdown
        safe --> [*]: shutdown
        ready --> [*]: arm
    }

    state Fly {
        [*] --> Hover
        Hover --> [*]: land
        Hover --> Move: cmd_js
        Move --> Hover: cmd_home
        Move --> Move: cmd_js
    }

    state Land {
        state ESC <<choice>>
        [*] --> ESC
        ESC --> ESC_up: ESC_down
        ESC --> ESC_down: ESC_up
        ESC_down --> hold
        ESC_up --> hold
        hold --> [*]: unarm
        hold --> [*]: takeoff
    }
```
