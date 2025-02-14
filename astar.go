package main

import (
	"container/heap" // Paket za implementaciju prioritetskog reda
	"fmt"
)

// Struktura čvora za A* algoritam
type Node struct {
	X, Y     int     // Koordinate čvora
	G, H, F  float64 // G: trošak od početka do trenutnog čvora, H: heuristika, F: ukupni trošak (G + H)
	Parent   *Node   // Pokazivač na roditeljski čvor za rekonstrukciju puta
	Index    int     // Indeks čvora u prioritetskom redu
	Walkable bool    // Označava je li čvor prohodan
	Weight   int
}

// DODANO - autor: Dino Gržinić
func calculateWeight(x, y int, weightType int) int {
	switch weightType {
	case 1:
		return 1
	case 2:
		return x + 1
	case 3:
		return y + 1
	case 4:
		return x + y + 2
	default:
		return 1
	}
}

// IZMJENA Autor - Ante Ujčić
// Implementacija A* algoritma
func aStarSearch(grid [][]*Node, start, goal *Node, animationSpeed int,
	heuristicFunc HeuristicFunc, diagonals bool) {
	openSet := &PriorityQueue{}
	heap.Init(openSet)

	start.G = 0
	start.H = heuristicFunc(start, goal)
	start.F = start.G + start.H
	heap.Push(openSet, start)

	closedSet := make(map[*Node]bool)
	pathFound := false

	for openSet.Len() > 0 {
		current := heap.Pop(openSet).(*Node)
		closedSet[current] = true
		printGrid(grid, start, goal, nil, openSet, closedSet, animationSpeed)

		// IZMJENA - autor: Marin Rabađija
		if current == goal {
			pathFound = true
			break
		}

		// Obrada susjednih čvorova
		for _, neighbor := range getNeighbors(grid, current, diagonals) {
			if !neighbor.Walkable || closedSet[neighbor] {
				continue
			}
			propG := current.G + float64(current.Weight)

			// Ako susjed nije u openSetu ili je pronađen bolji put
			if neighbor.Index == -1 && !inOpenSet(neighbor) {
				neighbor.G = propG
				neighbor.H = heuristicFunc(neighbor, goal)
				neighbor.F = neighbor.G + neighbor.H
				neighbor.Parent = current
				heap.Push(openSet, neighbor)
			} else if propG < neighbor.G {
				openSet.update(neighbor, propG, neighbor.H, current)
			}
		}
	}

	// DODANO - autor: Marin Rabađija
	if pathFound {
		pathLength := reconstructPath(grid, start, goal, openSet, closedSet, animationSpeed)
		fmt.Println("Duljina puta:", pathLength)
	} else {
		fmt.Println("Ne postoji put.")
	}
}

// Dohvaća susjedne čvorove
func getNeighbors(grid [][]*Node, node *Node, diagonals bool) []*Node {
	var directions [][2]int

	// IZMJENA - autor: Marin Rabađija
	// ako se koriste dijagonalna ili euklidska udaljenost
	if diagonals {
		directions = [][2]int{
			{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, -1}, {-1, 1}, {-1, -1}, {1, 1},
		}
	} else {
		directions = [][2]int{
			{1, 0}, {-1, 0}, {0, 1}, {0, -1},
		}
	}
	var neighbors []*Node

	for _, d := range directions {
		nx, ny := node.X+d[0], node.Y+d[1]
		if ny >= 0 && ny < len(grid) && nx >= 0 && nx < len(grid[0]) {
			neighbors = append(neighbors, grid[ny][nx])
		}
	}
	return neighbors
}

// Provjerava je li čvor u openSetu
func inOpenSet(node *Node) bool {
	return node.Index != -1
}

// DODAO autor: Ante Ujčić
// Funkcija za inicijalizaciju mreže
func initializeGrid(width, height, weightType int) [][]*Node {
	grid := make([][]*Node, height)
	for y := 0; y < height; y++ {
		grid[y] = make([]*Node, width)
		for x := 0; x < width; x++ {
			grid[y][x] = &Node{
				X:        x,
				Y:        y,
				Walkable: true,
				Index:    -1,
				Weight:   calculateWeight(x, y, weightType),
			}
		}
	}
	return grid
}

func main() {
	const width, height = 10, 10
	var weightType, mapSelection, startX, startY, goalX, goalY, animSpeed, heur int
	var grid [][]*Node
	var obstacles []position
	var start, goal *Node
	var heuristicFunc HeuristicFunc
	diagonals := false

	// Odabir načina inicijalizacije
	initMethod := chooseInitializationMethod()

	if initMethod == "1" {
		// Default način: učitavanje svih podataka iz input.txt
		weightType, mapSelection, startX, startY, goalX, goalY, animSpeed, heur = getInputFromFile()
		// Inicijalizacija mreže
		grid = initializeGrid(width, height, weightType)
		// Postavljanje prepreka prema odabiru mape
		obstacles = obstacleMaps[mapSelection-1]
		for _, obs := range obstacles {
			grid[obs.Y][obs.X].Walkable = false
		}
		heuristicFunc, diagonals = selectHeuristicFromValue(heur)

		// Označavanje početka i dodjela
		grid[startY][startX].Walkable = false
		start = grid[startY][startX]
		goal = grid[goalY][goalX]
	} else {
		// Ručni unos: interaktivno se postavljaju podaci
		weightType = getWeightType()
		// Inicijalizacija mreže
		grid = initializeGrid(width, height, weightType)
		// Unos prepreka (ručni odabir mape)
		obstacles = getSelectedMap()
		for _, obs := range obstacles {
			grid[obs.Y][obs.X].Walkable = false
		}
		heuristicFunc, diagonals = getSelectedHeuristic()

		// IZMJENA - autor: Marin Rabađija
		start = insertCoordinates(grid, height, width, 'A')
		goal = insertCoordinates(grid, height, width, 'B')
		// Ručni unos animacijske brzine
		animSpeed = insertAnimationSpeed()
		// U ručnom načinu možemo kasnije ručno odabrati heurističku funkciju:
		heur = 0
	}

	// Odabir heuristike
	// IZMJENA - autor: Marin Rabađija (dodan indikator za implementaciju dijagonala)
	aStarSearch(grid, start, goal, animSpeed, heuristicFunc, diagonals)
}
