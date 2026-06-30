## Setup

### Cloning
Since there are submodules, clone recursively
```
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git 
```
### Docker

Build docker image with 
```
docker build -t tardigrade-foxy -f docker/Dockerfile .
```

THen to run the image
```
docker run -it --rm -v $(pwd):/ws tardigrade-foxy bash
```