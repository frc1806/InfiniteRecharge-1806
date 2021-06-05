pipeline {
    agent any

    stages {
        stage('Build') {
	    tools{
	    	jdk "JDK11"
	    }
            steps {
                sh './gradlew build'
            }
        }
    }
}