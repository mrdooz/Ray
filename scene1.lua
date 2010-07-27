camera = {
	["pos"] = {0, 0, 0},
	["look-at"] = {0, 0, -300},		-- ignored for now
	["up"] = {0, 1, 0},
	["dist"] = 300,
}

materials = {
	{
		["name"] = "diffuse-red",
		["diffuse"] = {1, 0, 0},		-- rgb
		["kd"] = 0.1,
	}
}

scene = {
	{
		["name"] = "sphere0",
		["type"] = "sphere",
		["center"] = {-10, 0, -200},
		["radius"] = 10,
		["material"] = "diffuse-red"
	},

	{
		["name"] = "sphere1",
		["type"] = "sphere",
		["center"] = {10, 0, -200},
		["radius"] = 10,
		["material"] = "diffuse-red",
	},

	{
		["name"] = "sphere2",
		["type"] = "sphere",
		["center"] = {0, 0, -240},
		["radius"] = 10,
		["material"] = "diffuse-red",
	},

}
