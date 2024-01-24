# URUCHAMIANIE  DOCKERA

``` bash
docker run -dti --rm --name bitdock --net host --privileged -e DISPLAY=$DISPLAY -e XAUTHORITY=/tmp/.docker.xauth -v "/dev:/dev" -v "/tmp/.X11-unix:/tmp/.X11-unix" -v "/home/student/bitmap_ros/BM_HOME:/home/ros" bitmapros2
```
``` bash
xhost +local:docker
```
```bash
docker exec -it bitdock "/bin/bash"
```
# URUCHAMIANIE NODA
```bash
source /opt/ros/humble/setup.bash 
```
```bash
cd /home/ros/BM_ws
```
```bash
source install/setup.bash
```
```bash
ros2 launch src/bitmap/launch/bitmap.launch.py 
```

# Plik konfiguracyjny 
Plik konfiguracyjny znajduje się w poniżej opisanej lokalizacji: 
```bash
/home/ros/BM_ws/src/bitmap/bitmap/bitmap_setup.json
``` 
Zawartość pliku:
```json
{
    "bitmap_file": "/home/ros/mapp",
    "scaling":{
            "x": 3.883, 
            "y": 4.023
        },
    "dock": {
        "x": 2.00,     // m
        "y": 1.50,     // m
        "theta": 0.9, // stopnie
        "length": 22, // m
        "width": 18   // m
    },
    "tail": {
        "samples_limit": 100, // sample
        "visible": false
    }
}

``` 
`bitmap_file` - ścieżka do pliku z bitmapą

`scaling` - skalowanie w układzie X i Y

`dock` - konfiguracja docku. Dockiem można jeździć po ekranie używając klawiszy WSAD

`tail` - konfiguracaja "ogonka", który aktualnie podąża za dockiem. Można ustawić czy ma być wyświetlany czy nie.
# Sterowanie:
w trakcie działania programu upewnić się że aktywne jest okno z bitmapą następnie działają keybindy:
## wybór doku
klawisze numeryczne np. '1' '2'
## ruch dokiem
gdy wybrany jest dok do poruszania się służą klawisze 'w' 'a' 's' 'd' odpowiednio góra lewo dół prawo
## siatka
siatka jest włączana lub wyłączana klawiszem '['
## negatyw
negatyw jest włączany lub wyłączany klawiszem ']'
## Obrót
, wykonuje obrot w lewo
. wykonuje obrot w prawo
