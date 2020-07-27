# Exploración multirobot basada en lasegmentación del espacio
El trabajo desarrollado trata sobre el problema de exploración utilizando un grupo de robots, más específicamente sobrecomo lograr distribución eficiente de los robots en el espacio para una mayor coordinación en la exploración. Para esto, en lugar dedistribuir los robots sobre las fronteras entre el espacio conocido y el desconocido, como es usual, la propuesta apunta a identificarporciones del espacio similares a habitaciones o corredores, llamados segmentos, para luego distribuir los robots primero sobredichos segmentos y luego sobre las fronteras de los mismos.

## Como usar
Correr `.test_once [map_name] [starting_robot_number]` donde:
* `map_name` puede ser `labyrinth` o `office`
* `starting_robot` es un numero entre 1 y 5

Esto correspnde a explorar el entorno `map_name` con`starting_robot` robots.
