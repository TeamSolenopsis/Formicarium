
# Docker

Docker setup





## Links

[Docker install](https://www.docker.com/)\
[ROS GUI Docker](http://wiki.ros.org/docker/Tutorials/GUI)\
[Docker tutorial](https://docker-curriculum.com/#docker-images)\
[Docker repository](https://hub.docker.com/r/crazypandabier/solenopsis)\
[NetworkChuck VS Code](https://youtu.be/1ZfO149BJvg)\
[Dev containers](https://code.visualstudio.com/docs/devcontainers/attach-container)




## Setup image
Clone the repo

```bash
  git clone https://github.com/<username>/<repo>.git
  cd Docker 
```

Build the image

```bash
  docker build . -f Formicarium.Dockerfile -t solenopsis:tagname
```

Push the image to the solenopsis repository

```bash
  docker tag solenopsis:tagname crazypandabier/solenopsis:tagname
  docker login
  docker push crazypandabier/solenopsis:tagname
```
## Develop

### VS Code

#### setup

* Install the [docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
* Install [Remote development extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

#### use

pull the image from docker.io change tagname to the tag of the image

```bash
  docker pull crazypandabier/solenopsis:tagname
```

* Open VS Code
* Click open the docker extension
* In the images tab search for the pulled image.
* Right click on the image and click run interactive, container will start.
* Right click on the container in the containers tab
* Click on attach vs code

For some more help watch the video in the documentation section.

#### display

If you want to use gui application inside the docker container run these commands. change tagname to the tag of the image

```
$ xhost +
$ docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix crazypandabier/solenopsis:tagname
```

to stop xhost run 
```
$ xhost -
```

## FAQ //TODO

#### Question 1

Answer 1

#### Question 2

Answer 2
