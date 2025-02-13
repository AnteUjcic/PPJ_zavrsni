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

// DODAO Autor - Ante Ujčić
// HeuristicFunc predstavlja funkciju koja računa heuristiku između dva čvora.
type HeuristicFunc func(a, b *Node) float64

// Struktura čvora za A* algoritam
type Node struct {
	X, Y     int     // Koordinate čvora
	G, H, F  float64 // G: trošak od početka do trenutnog čvora, H: heuristika, F: ukupni trošak (G + H)
	Parent   *Node   // Pokazivač na roditeljski čvor za rekonstrukciju puta
	Index    int     // Indeks čvora u prioritetskom redu
	Walkable bool    // Označava je li čvor prohodan
	Weight   int
}

// DODANO - autor: Ante Ujčić
// Funkcija za odabir metode inicijalizacije (default ili ručni unos)
func chooseInitializationMethod() string {
	reader := bufio.NewReader(os.Stdin)
	for {
		fmt.Println("\nOdaberite način inicijalizacije:")
		fmt.Println("1. Default opcija (učitavanje koordinata iz input.txt datoteke)")
		fmt.Println("2. Ručni unos koordinata")
		fmt.Print("Vaš odabir: ")

		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		if input == "1" || input == "2" {
			return input
		}
		fmt.Println("Molimo unesite 1 ili 2.")
	}
}

// DODANO - autor: Ante Ujčić
// Funkcija čita početne i završne koordinate iz datoteke input.txt
func getCoordinatesFromFile(grid [][]*Node, height, width int) (*Node, *Node) {
	file, err := os.Open("input.txt")
	if err != nil {
		fmt.Println("Greška prilikom otvaranja datoteke input.txt:", err)
		os.Exit(1)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)

	// Učitavanje koordinata početka
	if !scanner.Scan() {
		fmt.Println("Nije moguće pročitati početne koordinate iz datoteke.")
		os.Exit(1)
	}
	line1 := scanner.Text()
	parts := strings.Fields(line1)
	if len(parts) < 2 {
		fmt.Println("Neispravan format početnih koordinata u datoteci.")
		os.Exit(1)
	}
	startX, err1 := strconv.Atoi(parts[0])
	startY, err2 := strconv.Atoi(parts[1])
	if err1 != nil || err2 != nil {
		fmt.Println("Neispravni brojevi za početne koordinate.")
		os.Exit(1)
	}
	// Provjera granica: (X mora biti manji od širine, Y manji od visine)
	if startX < 0 || startX >= width || startY < 0 || startY >= height {
		fmt.Println("Početne koordinate iz datoteke su izvan dozvoljenih granica.")
		os.Exit(1)
	}

	// Učitavanje koordinata cilja
	if !scanner.Scan() {
		fmt.Println("Nije moguće pročitati završne koordinate iz datoteke.")
		os.Exit(1)
	}
	line2 := scanner.Text()
	parts = strings.Fields(line2)
	if len(parts) < 2 {
		fmt.Println("Neispravan format završnih koordinata u datoteci.")
		os.Exit(1)
	}
	goalX, err1 := strconv.Atoi(parts[0])
	goalY, err2 := strconv.Atoi(parts[1])
	if err1 != nil || err2 != nil {
		fmt.Println("Neispravni brojevi za završne koordinate.")
		os.Exit(1)
	}
	if goalX < 0 || goalX >= width || goalY < 0 || goalY >= height {
		fmt.Println("Završne koordinate iz datoteke su izvan dozvoljenih granica.")
		os.Exit(1)
	}

	// Provjera da su čvorovi prohodni
	if !grid[startY][startX].Walkable {
		fmt.Println("Početna pozicija iz datoteke nije prohodna.")
		os.Exit(1)
	}
	if !grid[goalY][goalX].Walkable {
		fmt.Println("Završna pozicija iz datoteke nije prohodna.")
		os.Exit(1)
	}

	// Označavanje početka kao posebnog (ako to trebaš, npr. za neprohodnost)
	grid[startY][startX].Walkable = false

	start := grid[startY][startX]
	goal := grid[goalY][goalX]
	return start, goal
}

// DODANO - autor: Dino Gržinić
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

// DODANO - autor: Dino Gržinić
func getWeightType() int {
	reader := bufio.NewReader(os.Stdin)

	for {
		fmt.Println("\nOdaberite tip težine svakog čvora:")
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

// DODANO - autor: Ante Ujčić
func getInputFromFile() (int, int, int, int, int, int, int, int) {
	file, err := os.Open("input.txt")
	if err != nil {
		fmt.Println("Greška prilikom otvaranja datoteke input.txt:", err)
		os.Exit(1)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	var lines []string
	for scanner.Scan() {
		lines = append(lines, strings.TrimSpace(scanner.Text()))
	}
	if len(lines) < 6 {
		fmt.Println("Datoteka input.txt mora sadržavati najmanje 6 redaka!")
		os.Exit(1)
	}

	// 1. redak: tip težine čvora
	weightType, err := strconv.Atoi(lines[0])
	if err != nil || weightType < 1 || weightType > 4 {
		fmt.Println("Neispravna vrijednost za težinu čvora (mora biti 1-4).")
		os.Exit(1)
	}

	// 2. redak: odabir mape
	mapSelection, err := strconv.Atoi(lines[1])
	if err != nil || mapSelection < 1 || mapSelection > 3 {
		fmt.Println("Neispravna vrijednost za odabir mape (mora biti 1-3).")
		os.Exit(1)
	}

	// 3. redak: koordinate početka
	parts := strings.Fields(lines[2])
	if len(parts) < 2 {
		fmt.Println("Neispravan format početnih koordinata.")
		os.Exit(1)
	}
	startX, err1 := strconv.Atoi(parts[0])
	startY, err2 := strconv.Atoi(parts[1])
	if err1 != nil || err2 != nil {
		fmt.Println("Neispravne početne koordinate.")
		os.Exit(1)
	}

	// 4. redak: koordinate cilja
	parts = strings.Fields(lines[3])
	if len(parts) < 2 {
		fmt.Println("Neispravan format koordinata cilja.")
		os.Exit(1)
	}
	goalX, err1 := strconv.Atoi(parts[0])
	goalY, err2 := strconv.Atoi(parts[1])
	if err1 != nil || err2 != nil {
		fmt.Println("Neispravne koordinate cilja.")
		os.Exit(1)
	}

	// 5. redak: odabir brzine animacije
	animChoice, err := strconv.Atoi(lines[4])
	if err != nil || animChoice < 1 || animChoice > 3 {
		fmt.Println("Neispravan unos za brzinu animacije (mora biti 1-3).")
		os.Exit(1)
	}
	// Prema interaktivnom izboru: 1=Sporo (280 ms), 2=Srednje (180 ms), 3=Brzo (100 ms)
	var animationSpeed int
	switch animChoice {
	case 1:
		animationSpeed = 280
	case 2:
		animationSpeed = 180
	case 3:
		animationSpeed = 100
	}

	// 6. redak: odabir heurističke funkcije
	heurChoice, err := strconv.Atoi(lines[5])
	if err != nil || heurChoice < 1 || heurChoice > 3 {
		fmt.Println("Neispravan unos za heurističku funkciju (mora biti 1-3).")
		os.Exit(1)
	}

	return weightType, mapSelection, startX, startY, goalX, goalY, animationSpeed, heurChoice
}

// DODANO - autor: Ante Ujčić
// IZMJENA - autor: Marin Rabađija (dodan indikator za implementaciju dijagonala)
func selectHeuristicFromValue(choice int) (HeuristicFunc, bool) {
	switch choice {
	case 1:
		return manhattan, false
	case 2:
		return euclidean, true
	case 3:
		return diagonal, true
	default:
		// Ako se dogodi neispravan unos, default je Manhattan.
		return manhattan, false
	}
}

// DODAO Autor - Ante Ujčić
// Manhattan heuristika: zbraja apsolutne razlike koordinata
func manhattan(a, b *Node) float64 {
	return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
}

// DODAO Autor - Ante Ujčić
// Euklidska heuristika: izračunava pravokutnu udaljenost
func euclidean(a, b *Node) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return math.Sqrt(dx*dx + dy*dy)
}

// DODAO Autor - Ante Ujčić
// Dijagonalna heuristika: koristi kombinaciju ravnih i dijagonalnih pokreta
func diagonal(a, b *Node) float64 {
	dx := math.Abs(float64(a.X - b.X))
	dy := math.Abs(float64(a.Y - b.Y))
	D := 1.0
	D2 := math.Sqrt2
	return D*(dx+dy) + (D2-2*D)*math.Min(dx, dy)
}

// DODAO Autor - Ante Ujčić
// IZMJENA - autor: Marin Rabađija (dodan indikator za implementaciju dijagonala)
func getSelectedHeuristic() (HeuristicFunc, bool) {
	reader := bufio.NewReader(os.Stdin)
	for {
		fmt.Println("\nOdaberite heuristiku:")
		fmt.Println("1. Manhattan udaljenost")
		fmt.Println("2. Euklidska udaljenost")
		fmt.Println("3. Dijagonalna udaljenost")
		fmt.Print("Vaš odabir: ")

		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)
		choice, err := strconv.Atoi(input)
		if err != nil || choice < 1 || choice > 3 {
			fmt.Println("Molimo unesite broj između 1 i 3")
			continue
		}
		switch choice {
		case 1:
			return manhattan, false
		case 2:
			return euclidean, true
		case 3:
			return diagonal, true
		}
	}
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

// DODANO - autor: Marin Rabađija
func insertCoordinates(grid [][]*Node, height, width int, nodeType byte) *Node {
	reader1 := bufio.NewReader(os.Stdin)
	reader2 := bufio.NewReader(os.Stdin)

	if nodeType == 'A' {
		fmt.Println("\nUnesite koordinate POČETKA")
	} else {
		fmt.Println("\nUnesite koordinate CILJA")
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

// DODANO - autor: Marin Rabađija
func insertAnimationSpeed() int {
	reader := bufio.NewReader(os.Stdin)

	for {
		fmt.Println("\nOdaberite brzinu animacije:")
		fmt.Println("1. Sporo")
		fmt.Println("2. Srednje")
		fmt.Println("3. Brzo")
		fmt.Print("Vaš odabir: ")

		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		choice, err := strconv.Atoi(input)
		if err != nil || choice < 1 || choice > 3 {
			fmt.Print("Molimo unesite broj između 1 i 3\n\n")
		} else {
			animationSpeed := 0

			switch {
			case choice == 1:
				animationSpeed = 280
			case choice == 2:
				animationSpeed = 180
			case choice == 3:
				animationSpeed = 100
			}

			return animationSpeed
		}
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

func reconstructPath(grid [][]*Node, start, goal *Node, openSet *PriorityQueue,
	closedSet map[*Node]bool, animationSpeed int) int {
	var path []*Node
	var current = goal
	var counter = 0

	// Rekonstruira putanju od cilja do početka
	for current != nil {
		path = append(path, current)
		current = current.Parent
		counter += 1

		// IZMJENA - autor: Marin Rabađija
		printGrid(grid, start, goal, path, openSet, closedSet, animationSpeed)
	}
	return counter - 2
}

// Ispis mreže
func printGrid(grid [][]*Node, start, goal *Node, path []*Node,
	openSet *PriorityQueue, closedSet map[*Node]bool, animationSpeed int) {
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
	// ANSI escape code
	fmt.Print("\033[H\033[2J")

	for y := range grid {
		for x := range grid[y] {
			node := grid[y][x]

			//IZMJENA -  autor novih znakova za ispis: Marin Rabađija
			switch {
			case node == start:
				fmt.Print(Blue + "A " + Reset)
			case node == goal:
				fmt.Print(Blue + "B " + Reset)
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
	// IZMJENA - autor: Marin Rabađija
	time.Sleep(time.Duration(animationSpeed) * time.Millisecond)
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

// DODAO autor: Ante Ujčić
// Funkcija za provjeru i postavljanje početnih i ciljanih pozicija
func validateAndSetPositions(grid [][]*Node, startX, startY, goalX, goalY, width, height int) (*Node, *Node) {
	if startX < 0 || startX >= width || startY < 0 || startY >= height {
		fmt.Println("Početne koordinate su izvan raspona.")
		os.Exit(1)
	}
	if goalX < 0 || goalX >= width || goalY < 0 || goalY >= height {
		fmt.Println("Koordinate cilja su izvan raspona.")
		os.Exit(1)
	}
	if !grid[startY][startX].Walkable {
		fmt.Println("Početna pozicija nije prohodna.")
		os.Exit(1)
	}
	if !grid[goalY][goalX].Walkable {
		fmt.Println("Pozicija cilja nije prohodna.")
		os.Exit(1)
	}
	grid[startY][startX].Walkable = false
	return grid[startY][startX], grid[goalY][goalX]
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
		grid = initializeGrid(width, height, weightType)
		// Postavljanje prepreka prema odabiru mape
		obstacles = obstacleMaps[mapSelection-1]
		for _, obs := range obstacles {
			grid[obs.Y][obs.X].Walkable = false
		}
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
