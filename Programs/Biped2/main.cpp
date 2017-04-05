		biped->move_trajectory = false;
		
		// make prohibited area (moving)
		// 禁止エリアの座標，半径，速度
		biped->AddMoveArea(vec2_t(0.35, 0.4) , 0.2 , vec2_t(0.0,-0.1));
		biped->AddMoveArea(vec2_t(0.85, -0.85) , 0.2 , vec2_t(0.0,0.1));
		biped->AddMoveArea(vec2_t(1.2, 0.25) , 0.2 , vec2_t(-0.1,0.0));
		biped->AddMoveArea(vec2_t(0.05, -0.3) , 0.2 , vec2_t(0.05,0.0));

