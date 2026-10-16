extends RigidBody3D



### ----- Defining Variables Starts Here ----- ###
### █  █  ██  ███  ████  ██  ███  █    ████  ███
### █  █ █  █ █  █  ██  █  █ █  █ █    █    █   
### █  █ ████ ███   ██  ████ ███  █    ███   ██ 
###  ██  █  █ █ █   ██  █  █ █  █ █    █       █
###  ██  █  █ █  █ ████ █  █ ███  ████ ████ ███ 
### ----- Defining Variables Starts Here ----- ###


# Movement settings
var movement_speed: float = 1.0
var movement_force: float = 30.0

# Creating a reference slot for cameraLR
var CamPivot_LR: Node3D

# Creating a reference to FloorTrigger_Area3D & is_falling variable
var floor_trigger: Area3D
var is_falling: bool = false

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	# Get reference to CamPivot_LR where we base our movement direction of
	CamPivot_LR = get_node("CamPivotManager/CamPivot_RotateLeftRight")
	linear_damp = 4.0  # Add this - creates friction
	
	# Getting and storing reference for FloorTrigger
	floor_trigger = get_node("FloorTrigger_Area3D")
	
	
### ----- Defining Variables Ends Here ----- ###
### ----- Defining Variables Ends Here ----- ###



### ----- Physics Tick Sections Starts Here ----- #####
### ███  █  █ █  █  ███ ████  ███  ███    ████ ████  ███ █  █
### █  █ █  █ █  █ █     ██  █    █        ██   ██  █    █ █ 
### ███  ████  ██   ██   ██  █     ██      ██   ██  █    ██  
### █    █  █  ██     █  ██  █       █     ██   ██  █    █ █ 
### █    █  █  ██  ███  ████  ███ ███      ██  ████  ███ █  █
### ----- Physics Tick Sections Starts Here ----- #####


### Called every physics frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta: float) -> void:
	### ----- Checking Triggers for needed variables start here
	###  ███ █  █ ████  ███ █  █    ████ ███  ████  ███  ███ ████ ███   ███
	### █    █  █ █    █    █ █      ██  █  █  ██  █    █    █    █  █ █   
	### █    ████ ███  █    ██       ██  ███   ██  █ ██ █ ██ ███  ███   ██ 
	### █    █  █ █    █    █ █      ██  █ █   ██  █  █ █  █ █    █ █     █
	###  ███ █  █ ████  ███ █  █     ██  █  █ ████  ███  ███ ████ █  █ ███ 
	
		# Determine are we on floor or are we falling
	# Get all of the bodies that overlapp the floor_trigger
	var floortrig_overlaps = floor_trigger.get_overlapping_bodies()
	var floor_overlaps = []
	for overlaps in floortrig_overlaps:
		#print(overlaps)
		if "player_root" not in overlaps.name:
			floor_overlaps.append(overlaps)
		
		
	print(floor_overlaps)
	
	var floor_counts = floor_overlaps.size() # Then we count it
	if floor_counts > 0:
		is_falling = false
	else:
		is_falling = true
		
	print(floor_counts)
	print("is_falling = " + str(is_falling))
	
	
	
	### ----- Movement Input Starts Here ----- ###
	### █  █  ██  █  █ ████ █  █ ████ █  █ ████    ████ █  █ ███  █  █ ████
	### ████ █  █ █  █ █    ████ █    ██ █  ██      ██  ██ █ █  █ █  █  ██ 
	### █  █ █  █ █  █ ███  █  █ ███  █ ██  ██      ██  █ ██ ███  █  █  ██ 
	### █  █ █  █  ██  █    █  █ █    █  █  ██      ██  █  █ █    █  █  ██ 
	### █  █  ██   ██  ████ █  █ ████ █  █  ██     ████ █  █ █     ██   ██  
	## Defining Input Direction and set it first into 0 in every tick
	
	# First we set standard 2D Plane Movement
	var input_direction = Vector3.ZERO
	# Overriding Input Direction if movement keys are pressed
	if Input.is_action_pressed("MoveForward"):
		input_direction.z += 1.0
	if Input.is_action_pressed("MoveBackward"):
		input_direction.z -= 1.0
	if Input.is_action_pressed("MoveRight"):
		input_direction.x += 1.0
	if Input.is_action_pressed("MoveLeft"):
		input_direction.x -= 1.0
		
	# Then we set ascend direction wether its up like jumping, or down as crouching
	var ascend_direction: float = 0
	if Input.is_action_pressed("Jump"):
		ascend_direction += 1.0
	if Input.is_action_pressed("Crouch"):
		ascend_direction -= 1.0 
	
	### ---- Applying input into real action ----- ###
	###  ██  ███  ███  █    █  █ ████ █  █  ███    ████ █  █ ███  █  █ ████
	### █  █ █  █ █  █ █    █  █  ██  ██ █ █        ██  ██ █ █  █ █  █  ██ 
	### ████ ███  ███  █     ██   ██  █ ██ █ ██     ██  █ ██ ███  █  █  ██ 
	### █  █ █    █    █     ██   ██  █  █ █  █     ██  █  █ █    █  █  ██ 
	### █  █ █    █    ████  ██  ████ █  █  ███    ████ █  █ █     ██   ██ 
	### ---- Applying input into real action ----- ###
	
	# If there's input, apply movement
	if input_direction != Vector3.ZERO:
		# Normalize the input direction
		input_direction = input_direction.normalized()
		
		
		# Get forward and right vectors from CamPivot_LR
		var pivot_forward = -CamPivot_LR.global_transform.basis.z
		var pivot_right = CamPivot_LR.global_transform.basis.x
		
		# Calculate movement direction relative to CamPivot_LR
		var move_direction = (pivot_forward * input_direction.z) + (pivot_right * input_direction.x)
		move_direction = move_direction.normalized()
		
		# Apply central impulse for movement
		# We use delta to make it framerate-independent
		apply_central_impulse(move_direction * movement_force * delta)
		
		
