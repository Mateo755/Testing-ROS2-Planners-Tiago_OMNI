# Planners Testing – TIAGO OMNI Base ROS 2

Repozytorium służy do testowania różnych planerów ruchu w środowisku ROS 2 z użyciem robota TIAGo z bazą Omni. Skupiamy się na ocenie efektywności i niezawodności dostępnych planerów w środowisku symulacyjnym.

## 🎯 Cel projektu

- Uruchomienie symulacji robota Tiago Omni Base (https://github.com/pal-robotics/omni_base_simulation/tree/humble-devel) i test nawigacji na wczytanej mapie zajętości (https://docs.nav2.org/getting_started/index.html#running-the-example)   

- Konfiguracja i dostrojenie parametrów dla 4 różnych planerów ścieżki (https://docs.nav2.org/plugins/index.html#planners);  porównanie cech przetestowanych algorytmów; porównanie metryk: czasu planowania, długości i kształtu ścieżek uzyskanych dla różnych punktów docelowych;  

- Uruchomienie planner_benchmarking (https://github.com/ros-planning/navigation2/tree/main/tools/planner_benchmarking)


## 🤖 Środowisko bazowe

Symulacja oparta jest na [omni_base_simulation (gałąź humble-devel)](https://github.com/pal-robotics/omni_base_simulation/tree/humble-devel?tab=readme-ov-file) od PAL Robotics, która zawiera pełną konfigurację robota i środowiska.


