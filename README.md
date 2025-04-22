# Mini-prosjekt

I dette prosjektet skal vi:

- Styre en Quanzer Qube med ROS2
- Ha en simulator ved siden av
- En launch fil som kan bytte mellom Qube og simulator.
- Visualisering i RViz
- Styre vinkel via en terminal og/eller GUI.

#Mappestruktur
-Mini-prosjekt-main
--src
--README.md
---pid_controller_msgs
---qube_bringup
---qube_controller
---qube_description
---qube_driver

# Hvordan få prosjkete til å kjøre
1. last ned prosjekte som .zip eller gjennom git
2. åpne en terminal og installer følgende for ros2 "sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers"
3. gå til Mini-prosjekt-main/src/qube_bringup/launch og rediger bringup.launch.py og rediger linje 13 " f'source ~/qube_ws/install/setup.bash && '" og endre den til din filplassering
4. åpne en terminal og kjør "colcon build"
5. Deretter kjør "source install/setup.bash" i terminalen
6. så skriv "ros2 launch qube_bringup bringup.launch.py" i terminalen
7. Dertetter endre Fixed Frame til "world"
8. åpn en ny terminal og kjør "ls /dev/tty*" for å finne porten til quben
9. noter hvilken port du har og kjør "sudo chmod 666 /dev/ttyACMX" bytt ut ttyACMX med din port
10. nå skal du kunne gi en ny verdi eten gjennom set refrence viduet ditt eller gjenom RViz

#Simuleringsmodus 
om du kunn vil kjøre visualisering av pid cotrolleren må du:
1. gå til Mini-prosjekt-main/src/qube_bringup/urdf og rediger controlled_qube.urdf.xacro og gå til linje 7 og endre "<xacro:arg name="simulation" default="false" />" og endre til "<xacro:arg name="simulation" default="true" />"

#Oppdatere pid verdier
1. gå til "/Mini-prosjekt-main/src/qube_controller/qube_controller" og rediger linje 13 til 15 for å oppdatere kp, ki og kd
2. Di burde se slik ut orginalt:
       # PID-parametre
        self.declare_parameter('kp', 8.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
3. Endre til ønsket parmaeter og lagre og kjør prosjektet slik du ville ha gjort fra "Hvordan få prosjkete til å kjøre" fra linje 4   

   


 
