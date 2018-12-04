pipeline {
  agent any
  stages {
    stage('pull') {
      steps {
        sh 'echo "kernel pipeline test"'
      }
    }
    stage('') {
      steps {
        setGerritReview()
        setGerritReview(customUrl: 'review.gerrithub.io', unsuccessfulMessage: 'abc')
      }
    }
  }
}
