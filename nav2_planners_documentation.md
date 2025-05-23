# Porównanie planerów nawigacyjnych w ROS 2 (Nav2)

Ten dokument zawiera charakterystyki wybranych globalnych planerów dostępnych w Nav2 (ROS 2), wraz z zaleceniami ich użycia w zależności od typu robota i środowiska. Uwzględniono planery: NavFn (A\*, Dijkstra), Theta\*, SmacPlanner2D, SmacPlannerHybrid oraz SmacPlannerLattice.

---

## 👉 Przegląd planerów

### ✅ 1. NavFn Planner

* **Typ**: klasyczny planer siatki (Dijkstra lub A\*)
* **Implementacja**: `nav2_navfn_planner/NavfnPlanner`
* **Charakterystyka**:

  * Bardzo szybki
  * Nie uwzględnia orientacji końcowej
  * Generuje ścieżki o ostrych kątach ("po siatce")
* **Zastosowania**:

  * Proste mapy
  * Szybkie testy bazowe
  * Roboty holonomiczne

### ✅ 2. Theta\* Planner

* **Typ**: any-angle planer (ze skosami)
* **Implementacja**: `nav2_theta_star_planner/ThetaStarPlanner`
* **Charakterystyka**:

  * Gładsze ścieżki niż NavFn
  * Możliwość uwzględnienia orientacji (parametr)
  * Szybszy niż Smac, wolniejszy niż NavFn
* **Zastosowania**:

  * Otwarte środowiska
  * Potrzeba skracania dystansu
  * Roboty omni i diff-drive

### ✅ 3. SmacPlanner2D

* **Typ**: planowanie w siatce 2D z kosztami
* **Implementacja**: `nav2_smac_planner/SmacPlanner2D`
* **Charakterystyka**:

  * Uwzględnia koszty
  * Obsługuje wygładzanie ścieżek
  * Orientacja opcjonalna
* **Zastosowania**:

  * Gęste mapy z przeszkodami
  * Roboty holonomiczne

### ✅ 4. SmacPlannerHybrid

* **Typ**: SE2 Hybrid A\* (planowanie z orientacją)
* **Implementacja**: `nav2_smac_planner/SmacPlannerHybrid`
* **Charakterystyka**:

  * Pełne planowanie SE2 (z orientacją)
  * Obsługuje modele DUBIN, REEDS\_SHEPP
  * Bardzo dokładne, ale wolniejsze
* **Zastosowania**:

  * Roboty z ograniczeniami ruchu (Ackermann, car-like)
  * Wymagane precyzyjne dojazdy

### ✅ 5. SmacPlannerLattice

* **Typ**: planer oparty na trajektoriach (primitives)
* **Implementacja**: `nav2_smac_planner/SmacPlannerLattice`
* **Charakterystyka**:

  * Dokładny, szybki (jeśli dobrze skonfigurowany)
  * Wymaga pliku `lattice.graph`
  * Obsługuje orientację i ograniczenia ruchu
* **Zastosowania**:

  * Roboty AGV, przemysłowe
  * Gotowe bazy manewrów (katalog trajektorii)

---

## 🔁 Uwzględnianie kinematyki (SE(2))

Niektóre planery, takie jak SmacHybrid i SmacLattice, planują w przestrzeni **SE(2)** — czyli w pełnym układzie pozycji i orientacji:

```
SE(2) = (x, y, θ)
```

Gdzie:

* `x`, `y` — pozycja na mapie,
* `θ` (theta) — orientacja (yaw, czyli obrót wokół osi Z).

Planowanie w SE(2) uwzględnia:

* kierunek podejścia do celu,
* minimalny promień skrętu,
* brak możliwości skrętów w miejscu (dla niektórych robotów).

Planery 2D (NavFn, Theta\*, Smac2D) planują tylko w `(x, y)`, ignorując orientację.

📌 To istotne dla robotów nieholonomicznych – np.:

* Ackermann (samochodowe),

* roboty z zawężoną geometrią skrętu,

* wózki AGV.

---

## 📊 Porównanie cech planerów

| Planer          | Obsługa orientacji | Gładkość ścieżki | Koszty (inflacja) | Ograniczenia ruchu | Szybkość planowania | Typowe zastosowania          |
| ---------------- | ------------------ | ---------------- | ----------------- | ------------------ | ------------------- | ---------------------------- |
| NavFn (A\*)      | ❌ Nie              | ❌ Niska          | ❌ Nie             | ❌ Nie              | ✅✅✅ Bardzo szybki   | Proste mapy, roboty omni     |
| NavFn (Dijkstra) | ❌ Nie              | ❌ Niska          | ❌ Nie             | ❌ Nie              | ✅✅✅ Bardzo szybki   | Proste porównania, fallback  |
| Theta\*          | ⚠️ Opcjonalnie     | ✅ Umiarkowana    | ⚠️ Ograniczone    | ❌ Nie              | ✅✅ Szybki           | Skróty, otwarte przestrzenie |
| SmacPlanner2D    | ⚠️ Opcjonalnie     | ✅ Umiarkowana    | ✅ Tak             | ❌ Nie              | ✅✅ Szybki           | Gęste mapy, roboty omni      |
| SmacHybrid       | ✅ Tak (SE2)        | ✅✅ Bardzo dobra  | ✅✅ Tak            | ✅ Tak              | ⚠️ Wolniejszy       | Ackermann, roboty precyzyjne |
| SmacLattice      | ✅ Tak (SE2)        | ✅✅✅ Najlepsza    | ✅✅ Tak            | ✅✅ Zaawansowane    | ⚠️ Średnio szybki   | AGV, car-like, przemysłowe   |

---

## ⚙️ Parametry konfiguracyjne planerów

| Cecha / kolumna         | Parametry konfiguracyjne                                      | Opis                                                 |
| ----------------------- | ------------------------------------------------------------- | ---------------------------------------------------- |
| **Obsługa orientacji**  | `goal_heading_mode`, `use_final_approach_orientation`         | Czy planner uwzględnia `goal.pose.orientation`       |
|                         | `motion_model_for_search`                                     | Włącza model SE2 (np. DUBIN, REEDS\_SHEPP)           |
|                         | `goal.pose.orientation` (w kodzie)                            | Wymuszenie orientacji celu                           |
| **Gładkość ścieżki**    | `smooth_path`                                                 | Włączenie wygładzacza                                |
|                         | `smoother.{w_smooth, w_data, refinement_num, max_iterations}` | Regulacja poziomu wygładzania                        |
|                         | `non_straight_penalty`, `change_penalty`                      | Zachęcanie do prostych trajektorii                   |
| **Koszty (inflacja)**   | `cost_penalty`, `retrospective_penalty`                       | Kara za wchodzenie w droższe obszary                 |
|                         | `allow_unknown`                                               | Czy plan może prowadzić przez nieznane komórki       |
|                         | `cache_obstacle_heuristic`, `downsample_obstacle_heuristic`   | Optymalizacje obliczeń kosztów                       |
| **Ograniczenia ruchu**  | `motion_model_for_search`                                     | Model ruchu planowania SE2 (np. DUBIN = tylko przód) |
|                         | `minimum_turning_radius`                                      | Minimalny promień skrętu                             |
|                         | `reverse_penalty`, `change_penalty`, `non_straight_penalty`   | Kary za cofanie i zmiany kierunku                    |
|                         | *(dotyczy tylko SmacHybrid / Lattice)*                        | Planery 2D nie wspierają tych funkcji                |
| **Szybkość planowania** | `max_planning_time`, `max_iterations`                         | Limity czasowe i iteracyjne                          |
|                         | `downsample_costmap`, `downsampling_factor`                   | Planowanie z mniejszą rozdzielczością kosztmapy      |
|                         | `analytic_expansion_ratio`, `lookup_table_size`               | Złożoność obliczeń docięcia do celu                  |
|                         | `cache_obstacle_heuristic`                                    | Akceleracja na statycznych mapach                    |

---

## 📄 Uwagi praktyczne

* Parametry takie jak `cost_penalty`, `reverse_penalty`, `smooth_path` są kluczowe dla poprawy jakości ścieżki bez radykalnego wzrostu czasu obliczeń.
* Dobieraj `motion_model_for_search` zgodnie z rzeczywistymi możliwościami robota.
* Włączenie `cache_obstacle_heuristic` **drastycznie przyspiesza** ponowne planowanie.
* Dla testów w pustym środowisku nie ma potrzeby włączania zaawansowanej obsługi orientacji.

---

## 📄 Uwagi końcowe

* ❌ NavFn nie planuje orientacji i daje siatkowe, sztywne trasy.
* ✅ SmacHybrid i Lattice to najlepszy wybór do realistycznych robotów z ograniczeniami ruchu.
* ⚠️ Theta\* i Smac2D dają lepsze trasy niż NavFn przy podobnej szybkości.

---

Możesz dostroić parametry takich jak `cost_penalty`, `reverse_penalty`, `smooth_path`, aby lepiej dopasować zachowanie do konkretnego robota.

---

