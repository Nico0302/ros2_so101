.PHONY: deps
deps:
	cd ~/ros2_ws \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src -r -y

.PHONY: build
build:
	cd ~/ros2_ws \
    && colcon build --symlink-install

.PHONY: clean
clean:
	cd ~/ros2_ws \
	&& rm -rf build/ install/ log/