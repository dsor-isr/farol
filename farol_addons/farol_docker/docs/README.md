# Description

Folder with dockerfile and scripts to build an image and upload it to harbor and dockerhub. The image is then used for ci/cd of the FAROL stack.

## Relevant files

| Files                   | Description                                                                                                                                                                                               |
| ------                  | -----------                                                                                                                                                                                               |
| basic_entrypoint.sh     | entrypoiny for docker container, basically opens a bash shell and sources ROS environment.                                                                                                                |
| create_jenkins_image.sh | Script that builds a docker image to be used for ci/cd of the FAROL stack. It also uploads the docker image to and to dsor harbor and dockerhub.                                                          |
| Dockerfile              | Dockerfile with all the commands to a assemble the docker image to be used for ci/cd of the farol stack.                                                                                                  |
| install_requirements.sh | Install all the libraries and dependencies of the FAROL stack. Useful for preparing the system to run FAROL properly. Used when building docker images and preparing regular machines to run FAROL stack. |
