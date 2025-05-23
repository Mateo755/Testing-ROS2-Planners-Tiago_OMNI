# Instrukcja testowania planera w symulacji Nav2

Poniżej przedstawiono kroki do przetestowania planera za pomocą symulacji Nav2 w środowisku Gazebo.

## Krok 1: Źródłowanie środowiska

W katalogu głównym workspace'a (np. `~/ros2_ws`) wykonaj polecenie:

```bash
source install/setup.bash
```

Zapewni to dostęp do wszystkich zbudowanych pakietów i zależności.

## Krok 2: Uruchomienie symulacji i systemu Nav2

W pierwszym terminalu uruchom symulację Gazebo wraz z systemem Nav2:

```bash
ros2 launch omni_base_gazebo omni_base_gazebo.launch.py is_public_sim:=True navigation:=True
```

To polecenie uruchamia robota w symulacji oraz inicjalizuje cały system nawigacji Nav2.

## Krok 3: Uruchomienie testu planera

W drugim terminalu (również po wykonaniu komendy `source install/setup.bash`) uruchom test planera:

```bash
ros2 run planner_test test_planner
```

To polecenie rozpocznie proces testowy, który sprawdzi działanie planera w zadanej konfiguracji.

---

Upewnij się, że oba terminale działają w tym samym środowisku (`source install/setup.bash`) oraz że symulacja została poprawnie uruchomiona przed uruchomieniem testu planera.
