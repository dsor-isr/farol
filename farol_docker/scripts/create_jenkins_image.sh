LATEST_VERSION=v0.0.3

docker build --no-cache -t farol_jenkins:${LATEST_VERSION} .
docker tag farol_jenkins:${LATEST_VERSION} farol_jenkins:latest

# Login at harbor.dsor
docker login harbor.dsor.isr.tecnico.ulisboa.pt

# Add the version and latest tags to harbor
docker tag farol_jenkins:${LATEST_VERSION} harbor.dsor.isr.tecnico.ulisboa.pt/farol/farol_jenkins:${LATEST_VERSION}
docker push harbor.dsor.isr.tecnico.ulisboa.pt/farol/farol_jenkins:${LATEST_VERSION}
docker tag farol_jenkins:latest harbor.dsor.isr.tecnico.ulisboa.pt/farol/farol_jenkins:latest
docker push harbor.dsor.isr.tecnico.ulisboa.pt/farol/farol_jenkins:latest