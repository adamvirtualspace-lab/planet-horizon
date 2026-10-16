extends Node3D


# define camera_rotation vector2 value starts from 0
var camera_rotation: Vector2 = Vector2.ZERO

var mouse_sensitivity: float = 0.001
var max_y_rotation: float = 1.2

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta: float) -> void:
#	pass

# Capturing/decapturing Mouse
func _input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_cancel"):
		if Input.get_mouse_mode() ==Input.MOUSE_MODE_CAPTURED:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
			
	# If Mouse is captured, and assigning it into variable
	if event is InputEventMouseMotion:
		var mouse_event: Vector2 = event.screen_relative*mouse_sensitivity
		camera_look(mouse_event)
		
		
# Applying the Captured Mouse Movement variable
func camera_look(mouse_movement: Vector2) -> void:
	camera_rotation += mouse_movement
	
	transform.basis = Basis()
	
	# Get Immediate children
	var cam_pivot_lr = get_child(0)
	
	cam_pivot_lr.transform.basis = Basis()
	cam_pivot_lr.rotate_object_local(Vector3(0,1,0), -camera_rotation.x)
	
	var cam_pivot_ud = cam_pivot_lr.get_child(0)
	
	cam_pivot_ud.transform.basis = Basis()
	cam_pivot_ud.rotate_object_local(Vector3(1,0,0), -camera_rotation.y)
	
	#rotate_object_local(Vector3(0,1,0), -camera_rotation.x)
	#rotate_object_local(Vector3(1,0,0), camera_rotation.y)
	
	camera_rotation.y = clamp(camera_rotation.y, -max_y_rotation, max_y_rotation)
	
