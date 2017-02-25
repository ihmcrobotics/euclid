# Euclid

## Minutiae

### Tested Platforms
We test all of our software on OS X 10.11 El Capitan, Windows 7/8/10, and Ubuntu 14.04 LTS Desktop and Server.

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Test and documentation coverage
We have put our best effort into documenting and testing the entire library. Test coverage is above 90%. 

### Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

### Compatibility
This library is compatible with Java 8+.

### Dependency
This library sources depends on:
- the matrix library EJML [here](http://ejml.org/),
- the library for vector math Euclid Core [here](https://github.com/ihmcrobotics/euclid-core),

while the tests also on JUnit and PIT mutation testing library [here](http://pitest.org/).

## What is Euclid?
Euclid is a library that defines geometries such as lines and convex polygons, and also shapes such as cylinders and boxes.
The library is born from the need of having a base library for geometric applications that is well tested, flexible, and meant for real-time environment where
garbage generation is not allowed.
This library will be extended over the next months as the result of migrating progressively content from the IHMC open source software [here](https://github.com/ihmcrobotics/ihmc-open-robotics-software).

## How does Euclid work?
The idea is to facilitate the manipulation with geometries by providing classes for each type of geometry this library consider.
The user can then simply instantiate one of these classes and then interact with the geometry through the class API.
Examples will be provided as the library is being extended.

## Who would use Euclid?
Any software developer manipulating geometry objects or dealing with 2D or 3D graphical UI is susceptible to use this library as the base of a more complex algorithm.

## What is the goal of Euclid?
The goal for Euclid is to become the most flexible, easy to use, and fast library for geometry applications, so it results in great increase in development productivity. 

## How can I contribute to Euclid?
Please read [CONTRIBUTING.md](https://github.com/ihmcrobotics/euclid/blob/develop/CONTRIBUTING.md).

## Content
This section will be written as this library grows.

## Using Euclid from .jar releases with Maven/Gradle
The releases .jars for Euclid are hosted on Bintray.
You can browse the IHMC release packages at https://bintray.com/ihmcrobotics/maven-release.
Instructions for adding the Maven repository and identifying the artifacts can also be found on Bintray for each package.

At a minimum, you will need to have the following repository declared in your build script to use the Euclid .jars:

```gradle
repositories {
   maven {
      url  "http://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
   }

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}
```

Here is an example for adding the dependency to Euclid using your build script:
```gradle
dependencies {
   compile group: 'us.ihmc', name: 'euclid', version: '0.0'
}
```
