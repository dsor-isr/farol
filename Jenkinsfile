// Developed by: Marcelo Jacinto
// Date: 29/05/2022
pipeline {
    agent {
        docker {
            image 'docker.io/dsorisr/farol_jenkins:latest'
            //image 'harbor.dsor.isr.tecnico.ulisboa.pt/farol/farol:latest'
            //registryUrl 'https://harbor.dsor.isr.tecnico.ulisboa.pt'
            //registryCredentialsId 'harbor-robot-token'
            args '--entrypoint=""'
            reuseNode false
        }
    }
    environment {
        ROS_WORKSPACE="${WORKSPACE}/catkin_ws"
    }
    options {
        checkoutToSubdirectory('catkin_ws/src')
    }
    // Move all the packages to the default catkin workspace
    stages {
        // Build stage - compile the code (using 10 threads)
        stage('Build') {
            steps {
                echo 'Build..'
                dir('catkin_ws') {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    catkin build --no-status -j10'''
                }
            }
        }
        // Test stage - test the code
        stage('Test') {
            steps {
                echo 'Testing..'
                // Only test the code inside the farol meta-packages (ignoring 3rd party code)
                dir('catkin_ws/src/test') {
                    sh '''#!/bin/bash
                    bash unit_test.sh
                    '''
                }
            }
        }
        // Generate Doxygen documentation
        // only in release tags
        stage('Documentation') {
            when {
                expression {env.BRANCH_NAME == "main"}
            }
            steps{
                echo 'Generating Doxygen Documentation..'
                dir('catkin_ws/src') {
                    withCredentials([GitUsernamePassword(
                    credentialsId: 'github_app_tokn',
                    gitToolName: 'Default')]) 
                    {
                        sh '''#!/bin/bash
                        git config --global push.default tracking
                        python3 docs/scripts/generate_documentation.py deploy
                        '''
                    }
                }
            }
        }
        // Update the docker image on the cloud if the docker file has changed
        stage('Update Docker Image') {
            when {
                changeset "Dockerfile"
            }
            steps {
                echo 'Generating the new docker image'
                // TODO
                echo 'Uploading the new image to online hub'
                // TODO
            }
        }
    }
    // Cleanup the jenkins environment after running
    post {
        always {
            echo "Pipeline finished do cleanup"
            deleteDir()
        }
        success {
            echo "Release Success"
        }
        failure {
            echo "Release Failed"
        }
        cleanup {
            echo "Clean up in post work space"
            cleanWs()
        }
    }
}
