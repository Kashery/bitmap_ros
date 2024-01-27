# Uruchomienie Dockera
Poniżej zbiór komend pozwalający na uruchomienie dockera wraz z dostępem do wyświetlania:
``` bash
docker run -dti --rm --name bitdock --net host --privileged -e DISPLAY=$DISPLAY -e XAUTHORITY=/tmp/.docker.xauth -v "/dev:/dev" -v "/tmp/.X11-unix:/tmp/.X11-unix" -v "/home/student/bitmap_ros/BM_HOME:/home/ros" bitmapros2
```
``` bash
xhost +local:docker
```
```bash
docker exec -it bitdock "/bin/bash"
```
# Uruchomienie noda
Lista koment pozwalająca na uruchomienie noda `bitmap`:
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
        "theta": 1, // stopnie
        "length": 22, // cm
        "width": 18   // cm
    },
    "tail": {
        "samples_limit": 100, // sample
        "visible": false
    }
}

``` 
* `bitmap_file` - ścieżka do pliku z bitmapą,

* `scaling` - skalowanie w układzie X i Y,

* `dock` - konfiguracja doku. Dokiem można sterowac w uruchomionym oknie,

* `tail` - konfiguracaja "ogonka", który subsktybuje dane typu `Pose2D`.

# Opis sterowania dokami przy użyciu klawiszy
W trakcie działania programu upewnić się że aktywne jest okno z bitmapą. Poniżej zostanie opisane sterowanie dokami:
## Wybór doku
* `1` - Wybiera dok pierwszy
* `2` - Wybiera dok drugi
## Poruszanie się dokiem
* `w` - ruch w górę
* `s` - ruch w dół
* `a` - ruch w lewo
* `d` - ruch w prawo
* `,` - obrot w lewo
* `.` - obrot w prawo
## Siatka
Klawisz `[` pozwala właczyć lub wyłączyć siatkę 
## Negatyw
Klawisz `]` pozwala właczyć lub wyłączyć kolory w negatywie

# Subskrybowanie pozycji i wyświetlanie ogonka
Istnieje możliwość przechwycenia informacji na temat pozycji z publishera i wyświetlenia ich na ekranie. Node `bitmap` zwiera wbudowany w siebie subscriber, który pozwala na wyświetlanie pozycji w postaci kropki oraz pozostawiania "ogonka" na ekranie gdy publikowana pozycja ulega zmianie. Publisher i subscriber obsłgują dane typu `geometry_msgs/msg/Pose2D`. 

Aby wyświetlić te dane na ekranie stworzony publisher zawierający pozycję `[x,y]` oraz orientację `theta` musi mieć nazwę `pos_track` i być typu `Pose2D`.
