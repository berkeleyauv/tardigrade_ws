### Docker

Build docker image with 
```
docker build -t tardigrade-foxy -f docker/Dockerfile .
```

THen to run the image
```
docker run -it --rm -v $(pwd):/ws tardigrade-foxy bash
```