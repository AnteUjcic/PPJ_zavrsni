package main

// PREBAČENO - autor: Dino Gržinić

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
)

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
