
proc load_map { mapName } {
	tracelog "loading $mapName"
	switch -exact -- $mapName {
		hub {
			load_map_slices $mapName {assets/hub.png} {
				{ 1 1 0 } { teleport map00 }
				{ 2 1 0 } { teleport map01 }
				{ 3 1 0 } { teleport map02 }
			}
		}
		map00 {
			load_map_slices mapName assets/map01.png
		}
		map01 {
			load_map_db mapName assets/test.db
		}
		map02 {
			load_map_heightfield mapName
		}
	}
}

teleport hub
