# Nav2 Configuration Documentation Links

Poniżej znajdują się linki do dokumentacji konfiguracyjnej dla różnych planerów Nav2:

* [Konfiguracja NavFn](https://docs.nav2.org/configuration/packages/configuring-navfn.html#parameters)
* [Konfiguracja Smac 2D](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html)
* [Konfiguracja Smac Hybrid](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
* [Konfiguracja Smac Lattice](https://docs.nav2.org/configuration/packages/smac/configuring-smac-lattice.html)
* [Konfiguracja Theta\*](https://docs.nav2.org/configuration/packages/configuring-thetastar.html#parameters)

## Przydatne komendy pomocnicze

* Wyświetlenie aktualnie wybranego planera:

  ```bash
  ros2 param get /planner_server planner_plugins
  ```
* Pobranie listy wszystkich parametrów planera:

  ```bash
  ros2 param list /planner_server
  ```
* Sprawdzenie wartości konkretnego parametru:

  ```bash
  ros2 param get /planner_server <nazwa_parametru>
  ```
