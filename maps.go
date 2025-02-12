package main

// DODANO - autor: Dino Gržinić

type position struct {
	X int
	Y int
}

var obstacleMaps = [][]position{
	// Mapa 1 - Jednostavna linija u sredini
	{
		position{4, 3},
		position{4, 4},
		position{4, 5},
		position{4, 6},
	},

	// Mapa 2 - Dva L zida
	{
		position{2, 2},
		position{2, 3},
		position{2, 4},
		position{3, 4},
		position{4, 4},
		position{6, 6},
		position{6, 7},
		position{6, 8},
		position{7, 6},
		position{8, 6},
	},

	// Mapa 3 - Uzorak labirinta
	{
		position{1, 1},
		position{2, 1},
		position{3, 1},
		position{3, 2},
		position{3, 3},
		position{1, 3},
		position{1, 4},
		position{1, 5},
		position{3, 5},
		position{4, 5},
		position{5, 5},
		position{7, 1},
		position{7, 2},
		position{7, 3},
		position{7, 4},
		position{7, 5},
		position{5, 7},
		position{6, 7},
		position{7, 7},
		position{8, 7},
	},
}
