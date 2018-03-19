# move4d_ros_lib

move4d_ros_lib provide an interface between the core of move4d libraries such
as the modification of robot positions, the creation and initialisation of the
world representation and the modules of move4d.

## building

It requires to have a recent versions of move4d libraries installed. It will
search for them under the `${ROBOTPKG_BASE}` directory, so that environment
variable must be set properly.

See the [documentation of move4d][move4d-doc] for instructions on the
installation of these libraries (you don't need the move4d executable,
only the libraries: libmove3d, move4d; check the
specific installation instructions for ROS).

move4d_ros_lib is a catkin package, so build it using `catkin_make install` in
your catkin workspace.

[move4d-doc]: https://redmine.laas.fr/projects/move3d/wiki/Wiki ""

## Usage

TODO

You can refer to the documentation in the headers, and to move4d_facts package as an example.

## Documentation

You can build the API documentation with doxygen and the provided Doxyfile, or consult it [here][doc]

[doc]: https://laas.github.io/move4d_ros_lib/ "API documentation of move4d_ros_lib"
