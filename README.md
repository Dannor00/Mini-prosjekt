# Mini-prosjekt
Dette prosjektet er en del av ROS2-labøvingene, og kombinerer alt vi har lært i lab 1, 2 og 3. Målet er å styre en Quanser Qube både i ekte og simulert miljø, med støtte for visualisering og styring via GUI eller terminal.
I dette prosjektet skal vi:

- Styre en Quanser Qube med ROS2  
- Ha en simulator ved siden av  
- En launch-fil som kan bytte mellom Qube og simulator  
- Visualisering i RViz  
- Styre vinkel via en terminal og/eller GUI  

---
## Mappestruktur

<pre lang="bash"><code>Mini-prosjekt-main/└── src/ ├── <b>qube_description/</b> # Geometrisk beskrivelse av Quben (URDF/XACRO)
                            ├── <b>qube_driver/</b> # Hardware interface (ROS 2 Control mot ekte Qube) 
                            ├── <b>qube_bringup/</b> # Launch- og konfigurasjonsfiler 
                            ├── <b>qube_controller/</b> # PID-regulator som bruker /joint_states 
                            └── <b>pid_controller_msgs/</b> # Egendefinert service for referanseverdi (SetReference.srv) </code></pre> 
                                         



## Hvordan få prosjektet til å kjøre

1. Last ned prosjektet som `.zip` eller via Git:

    ```bash
    git clone --recurse-submodules git@github.com:Dannor00/Mini-prosjekt.git
    ```
   Ved .zip nedlasting, er det nødvendig å kjøre:
  
    ```bash
      git submodule update --init --recursive
    ```
   inne i repo folderen for å laste ned qube_driver filene.

2. Åpne en terminal og installer nødvendige ROS2-pakker:

    ```bash
    sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
    ```

3. Gå til `Mini-prosjekt-main/src/qube_bringup/launch` og rediger `bringup.launch.py` om nødvendig.  
   Endre linje 13 og 20 til:

    ```python
    f'source ~/<path_to_repo>/Mini-prosjekt/install/setup.bash && '
    ```

4. Åpne en terminal og bygg prosjektet:

    ```bash
    colcon build
    ```

5. Kjør deretter:

    ```bash
    source install/setup.bash
    ```

6. Start systemet:

    ```bash
    ros2 launch qube_bringup bringup.launch.py
    ```

7. Åpne RViz og sett `Fixed Frame` til:

    ```
    world
    ```

8. Åpne en ny terminal og kjør:

    ```bash
    ls /dev/tty*
    ```

    for å finne porten til Quben (f.eks. `/dev/ttyACM0`).

9. Kjør deretter:

    ```bash
    sudo chmod 666 /dev/ttyACM0
    ```

    Husk å bytte ut `ttyACM0` med riktig port.

10. Nå skal du kunne sette en ny referanseverdi via terminal eller rqt ved å endre verdien i service "/set_refernece".

---

## Simuleringsmodus

Hvis du kun vil kjøre simulering og visualisering av PID-kontrolleren:

1. Gå til:

    ```
    src/qube_bringup/urdf/controlled_qube.urdf.xacro
    ```

2. Endre linje 7 fra:

    ```xml
    <xacro:arg name="simulation" default="false" />
    ```

    til:

    ```xml
    <xacro:arg name="simulation" default="true" />
    ```

---

## Oppdatere PID-verdier i kildekoden

1. Gå til:

    ```
    src/qube_controller/qube_controller/
    ```

2. Rediger linje 13–15 for å oppdatere `kp`, `ki` og `kd`. De ser slik ut:

    ```python
    self.declare_parameter('kp', 8.0)
    self.declare_parameter('ki', 0.0)
    self.declare_parameter('kd', 0.1)
    ```

3. Endre til ønskede verdier, lagre, og start prosjektet som beskrevet over fra punkt 4.

---

om du vil bare teste ulike PID parmaneter vil vi heller anbefale at du tester med:

```bash
ros2 param set /pid_controller_node k<ønsket parameter> <en 'dobbel' verdi >= 0>
```
for de ulike parameterne.  

#NB!
Optimale PID-verdier er avhengig om programmet kjører i simuleringmodus eller på fysisk qube. Simulerings verdier vil funger med lave PID-verdier, mens fysisk qube behøver høyere PID-verdier for å kunne gi utslag.  
