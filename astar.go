package main

import (
	"bufio"
	"container/heap" // Paket za implementaciju prioritetskog reda
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
	"time"
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

// Heuristička funkcija (Manhattan udaljenost)
func heuristic(a, b *Node) float64 {
	return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
}

// autor: Dino Gržinić
func getSelectedMap() []position {
	reader := bufio.NewReader(os.Stdin)

	for {
		fmt.Print("\nOdaberite mapu (1-3): ")
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		mapNum, err := strconv.Atoi(input)
		if err != nil || mapNum < 1 || mapNum > 3 {
			fmt.Println("Unesite broj od 1 do 3")
			continue
		}

		return obstacleMaps[mapNum-1]
	}
}

// autor: Dino Gržinić
func getWeightType() int {
	reader := bufio.NewReader(os.Stdin)

	for {
		fmt.Println("Odaberite tip težine svakog čvora:")
		fmt.Println("1. Svi čvorovi imaju težinu 1")
		fmt.Println("2. Težina = broj stupaca")
		fmt.Println("3. Težina = broj redaka")
		fmt.Println("4. Težina = retci + stupci")
		fmt.Print("Vaš odabir: ")

		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		weightType, err := strconv.Atoi(input)
		if err != nil || weightType < 1 || weightType > 4 {
			fmt.Print("Molimo unesite broj između 1 i 4\n\n")
			continue
		}
		return weightType
	}
}

// autor: Dino Gržinić
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

func insertCoordinates(grid [][]*Node, height, width int, nodeType byte) *Node {
	reader1 := bufio.NewReader(os.Stdin)
	reader2 := bufio.NewReader(os.Stdin)

	if nodeType == 'A' {
		fmt.Println("\nUnesite koordinate početka")
	} else {
		fmt.Println("\nUnesite koordinate cilja")
	}

	for {
		fmt.Print("X-koordinata: ")
		inputX, _ := reader1.ReadString('\n')
		inputX = strings.TrimSpace(inputX)
		x, errX := strconv.Atoi(inputX)

		fmt.Print("Y-koordinata: ")
		inputY, _ := reader2.ReadString('\n')
		inputY = strings.TrimSpace(inputY)
		y, errY := strconv.Atoi(inputY)

		if errX != nil || errY != nil {
			fmt.Println("Molimo unesite brojeve")
		} else if x < 0 || x >= height || y < 0 || y >= width {
			fmt.Println("X-koordinate moraju biti u rasponu od 0 do", height-1)
			fmt.Println("Y-koordinate moraju biti u rasponu od 0 do", width-1)
		} else if !grid[y][x].Walkable {
			fmt.Println("Navedene koordinate su zauzete!")
		} else {
			if nodeType == 'A' {
				grid[y][x].Walkable = false
			}
			return grid[y][x]
		}
		println()
	}
}

// Implementacija A* algoritma
func aStarSearch(grid [][]*Node, start, goal *Node) {
	openSet := &PriorityQueue{}
	heap.Init(openSet)

	start.G = 0
	start.H = heuristic(start, goal)
	start.F = start.G + start.H
	heap.Push(openSet, start)

	closedSet := make(map[*Node]bool)
	pathFound := false

	for openSet.Len() > 0 {
		current := heap.Pop(openSet).(*Node)
		closedSet[current] = true

		// Ako je cilj dosegnut, rekonstruiraj put
		if current == goal {
			pathFound = true
			break
		}

		// Obrada susjednih čvorova
		for _, neighbor := range getNeighbors(grid, current) {
			if !neighbor.Walkable || closedSet[neighbor] {
				continue
			}
			propG := current.G + float64(current.Weight)
			// Ako susjed nije u openSetu ili je pronađen bolji put
			if neighbor.Index == -1 && !inOpenSet(neighbor) {
				neighbor.G = propG
				neighbor.H = heuristic(neighbor, goal)
				neighbor.F = neighbor.G + neighbor.H
				neighbor.Parent = current
				heap.Push(openSet, neighbor)
			} else if propG < neighbor.G {
				openSet.update(neighbor, propG, neighbor.H, current)
			}
		}

		printGrid(grid, start, goal, nil, openSet, closedSet)
	}

	// Ispis rezultata
	if pathFound {
		pathLength := reconstructPath(grid, start, goal, openSet, closedSet)
		fmt.Println("Duljina puta:", pathLength)
	} else {
		fmt.Println("Ne postoji put.")
	}
}

// Dohvaća susjedne čvorove
func getNeighbors(grid [][]*Node, node *Node) []*Node {
	directions := [][2]int{
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},
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

func reconstructPath(grid [][]*Node, start, goal *Node, openSet *PriorityQueue, closedSet map[*Node]bool) int {
	var path []*Node
	var current = goal
	var counter = 0

	// Rekonstruira putanju od cilja do početka
	for current != nil {
		path = append(path, current)
		current = current.Parent
		counter += 1

		printGrid(grid, start, goal, path, openSet, closedSet)
	}
	return counter - 2
}

// Ispis mreže
func printGrid(grid [][]*Node, start, goal *Node, path []*Node, openSet *PriorityQueue, closedSet map[*Node]bool) {
	// Ispis vizualizacije
	pathMap := make(map[*Node]bool)
	for _, p := range path {
		pathMap[p] = true
	}

	openSetMap := make(map[*Node]bool)
	if openSet != nil {
		for _, n := range *openSet {
			openSetMap[n] = true
		}
	}

	Reset := "\033[0m"
	Red := "\033[31m"
	Green := "\033[32m"
	Blue := "\033[0;34m"
	Yellow := "\033[33m"
	fmt.Print("\033[H\033[2J")

	for y := range grid {
		for x := range grid[y] {
			node := grid[y][x]

			switch {
			case node == start:
				fmt.Print(Blue + "S " + Reset)
			case node == goal:
				fmt.Print(Blue + "G " + Reset)
			case pathMap[node]:
				fmt.Print(Green + "0 " + Reset) // Put
			case !node.Walkable:
				fmt.Print("# " + Reset) // Prepreka
			case openSet != nil && openSetMap[node]:
				fmt.Print(Yellow + "* " + Reset) // u setu
			case closedSet != nil && closedSet[node]:
				fmt.Print(Red + "* " + Reset) // zatvoren/posjećen
			default:
				fmt.Print(". ")
			}
		}
		fmt.Println()
	}
	fmt.Println()
	time.Sleep(150 * time.Millisecond)
}

func main() {
	// Definicija dimenzija mreže
	width, height := 10, 10
	weightType := getWeightType()

	// Inicijalizacija mreže čvorova
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

	// Definicija prepreka na mreži
	obstacles := getSelectedMap()

	// Označavanje prepreka kao neprohodnih čvorova
	for _, obs := range obstacles {
		grid[obs.Y][obs.X].Walkable = false
	}

	start := insertCoordinates(grid, height, width, 'A')
	goal := insertCoordinates(grid, height, width, 'B')

	// Pokretanje A* pretrage
	aStarSearch(grid, start, goal)
}
