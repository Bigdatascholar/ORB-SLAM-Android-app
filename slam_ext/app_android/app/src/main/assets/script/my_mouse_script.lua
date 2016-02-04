-- /asset/script/my_mouse_script.lua
my_mouse_script.fov = 60.0;

function my_mouse_script:start(node)
  print(node.name)
end

function my_mouse_script:update(node)
  node:getTransform().matrix:appendTranslation(.01,0,0);
end

function my_mouse_script:stop()
	print("STOP")
end