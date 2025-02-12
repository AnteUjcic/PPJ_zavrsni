package main

import (
	"container/heap" // Paket za implementaciju prioritetskog reda
	"fmt"
	"os"
)

// Struktura čvora za A* algoritam
type Node struct {
	X, Y     int     // Koordinate čvora
	G, H, F  float64 // G: trošak od početka do trenutnog čvora, H: heuristika, F: ukupni trošak (G + H)
	Parent   *Node   // Pokazivač na roditeljski čvor za rekonstrukciju puta
	Index    int     // Indeks čvora u prioritetskom redu
	Walkable bool    // Označava je li čvor prohodan
	Weight   int     // Težina čvora
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

// IZMJENIO Autor - Ante Ujčić
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

func main() {
	const width, height = 10, 10
	var weightType, mapSelection, startX, startY, goalX, goalY, animSpeed, heur int
	var grid [][]*Node
	var obstacles []position
	var start, goal *Node

	// Odabir načina inicijalizacije
	initMethod := chooseInitializationMethod()

	if initMethod == "1" {
		// Default način: učitavanje svih podataka iz input.txt
		weightType, mapSelection, startX, startY, goalX, goalY, animSpeed, heur = getInputFromFile()
		// Inicijalizacija mreže
		grid = make([][]*Node, height)
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
		// Postavljanje prepreka prema odabiru mape
		obstacles = obstacleMaps[mapSelection-1]
		for _, obs := range obstacles {
			grid[obs.Y][obs.X].Walkable = false
		}
		// Provjera granica i prohodnosti učitanih koordinata
		if startX < 0 || startX >= width || startY < 0 || startY >= height {
			fmt.Println("Početne koordinate iz datoteke su izvan raspona.")
			os.Exit(1)
		}
		if goalX < 0 || goalX >= width || goalY < 0 || goalY >= height {
			fmt.Println("Koordinate cilja iz datoteke su izvan raspona.")
			os.Exit(1)
		}
		if !grid[startY][startX].Walkable {
			fmt.Println("Početna pozicija iz datoteke nije prohodna.")
			os.Exit(1)
		}
		if !grid[goalY][goalX].Walkable {
			fmt.Println("Pozicija cilja iz datoteke nije prohodna.")
			os.Exit(1)
		}
		// Označavanje početka i dodjela
		grid[startY][startX].Walkable = false
		start = grid[startY][startX]
		goal = grid[goalY][goalX]
	} else {
		// IZMJENA - autor: Dino Gržinić
		// Ručni unos: interaktivno se postavljaju podaci
		weightType = getWeightType()
		// Inicijalizacija mreže
		grid = make([][]*Node, height)
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

		// IZMJENA - autor: Dino Gržinić
		// Unos prepreka (ručni odabir mape)
		obstacles = getSelectedMap()
		for _, obs := range obstacles {
			grid[obs.Y][obs.X].Walkable = false
		}

		// Ručni unos početnih i završnih koordinata
		start = insertCoordinates(grid, height, width, 'A')
		goal = insertCoordinates(grid, height, width, 'B')

		// Ručni unos animacijske brzine
		animSpeed = insertAnimationSpeed()
		// U ručnom načinu možemo kasnije ručno odabrati heurističku funkciju:
		heur = 0
	}

	// Postavljanje animacijske brzine – sada samo koristimo vrijednost iz varijable animSpeed
	animationSpeed := animSpeed
	diagonals := false

	// Odabir heuristike
	// IZMJENA - autor: Marin Rabađija (dodan indikator za implementaciju dijagonala)
	var heuristicFunc HeuristicFunc
	if initMethod == "1" {
		heuristicFunc, diagonals = selectHeuristicFromValue(heur)
	} else {
		heuristicFunc, diagonals = getSelectedHeuristic()
	}

	aStarSearch(grid, start, goal, animationSpeed, heuristicFunc, diagonals)
}
