# 2024-group-18

[![pipeline status](https://git.chalmers.se/courses/dit638/students/2024-group-18/badges/main/pipeline.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-18/-/commits/main)
[![coverage report](https://git.chalmers.se/courses/dit638/students/2024-group-18/badges/main/coverage.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-18/-/commits/main)
[![Latest Release](https://git.chalmers.se/courses/dit638/students/2024-group-18/-/badges/release.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-18/-/releases)


## Prerequisities

In this project, the tools were installed in `Ubuntu 22.04 LTS`. This can be done through a local boot, subsystem, multipass, virtual box, etc.

Once ubuntu is running, you will need the following technologies:
- `git` 2.34.1, as well as having a public SSH key set up already
- `g++` 11.4.0
- `cmake` 2.22.1
- `make` 4.3
- your preferred text editor/IDE if you wish to edit the C++ files, header files, or Dockerfile

## First Steps

1. Clone the repo by running the following command `git clone git@git.chalmers.se:courses/dit638/students/2024-group-18.git`
2. Move into into the *src-repo-folder* that you just cloned by running `cd 2024-group-18/src`
3. Create a build folder by running `mkdir build`.
4. Move into the build folder by running `cd build`.
5. Generate the build files by running `cmake ..`
6. Build the project by running `make`

Once those steps are done, if everything is correct, you should see this as the output of the last command:

```bash
[ 16%] Building CXX object CMakeFiles/helloworld.dir/helloworld.cpp.o
[ 33%] Building CXX object CMakeFiles/helloworld.dir/PrimeChecker.cpp.o
[ 50%] Linking CXX executable helloworld
[ 50%] Built target helloworld
[ 66%] Building CXX object CMakeFiles/helloworld-Runner.dir/TestPrimeChecker.cpp.o
[ 83%] Building CXX object CMakeFiles/helloworld-Runner.dir/PrimeChecker.cpp.o
[100%] Linking CXX executable helloworld-Runner
[100%] Built target helloworld-Runner
```

### Test the project:

```bash
./helloworld 123
```

OUTPUT
```bash
Group, 18; 123 is a prime number? 0
```

### Execute the tests:

```bash
./helloworld-Runner
```

OUTPUT
```c
===============================================================================
All tests passed (1 assertion in 1 test case)
```

## Contribution guide
There are two ways to contribute:
1. Adding new features
2. Fixing unexpected behavior in existing features

### Adding new features

Step 1: Create an issue for it, following a specified template and adding appropiate tags.
 
Step 2: Associate the feature with requirement(s) or user stories and define acceptance criteria.

Step 3: Assign yourself as the contributor for the issue and create a new branch off of main for the feature, giving it an appropriate/related name.

Step 4: Work on the corresponding feature and commit on the newly created branch by following the [commit template](#commit-template).

Step 5: Open a merge request, and request a code review from someone who wasn't a contributor on the feature.

Step 6: If there are any significant changes that need to be done - fix them, then ask for a review again until there are no more issues and the merge request is approved.

Step 7: Merge the branch into the main branch and solve any merge conflicts that may arise.

### Bug report

Step 1: Create an issue for it, following a specified template and adding appropiate tags.

Step 2: Provide information about the software version, operating system, and platform you are using.

Step 3: Provide replication steps and more information about the problem if requested from a contributor.

### Issue creation & the associated branch

#### Issue
- Title: Concise, giving information about the "what?" of the feature
- Description: Giving information about the associated requirements, user stories, and acceptance criteria
- Labels: At a glance categorisation
- Assignee: The person working on the corresponding issue
- Deadline: If necessary

#### Branch
- Have issue number at the beginning
- The name should be in kebab case (e.g. "1-feature-x")

### Code review practices

When doing code reviews, we follow the guidelines from this [post](https://phauer.com/2018/code-review-guidelines/) from Phillip Hauer's blog. To summarize the points from there:

As an author, it is important to understand that mistakes are normal, and any criticism on your code is not directed at you. The team is trying to create a great product, and the reviewer can offer new perspectives on your code.

As a reviewer, formulate your feedback as I-statements (I think/believe etc.) instead of you-statements. Referencing your own person and the code rather than the author can help with the impact of criticism. Also, remember that there are always more solutions to a problem, and it's not worth it sometimes to request a change.

## Commit template

The following is a reference material that ensures cohesion between commit messages. The format used is inspired by and uses principles from the lecture "Commits and Code Reviews" (credit to Francisco Gomes de Oliveira Neto) and the ["How to Write a Git Commit Message" article](https://chris.beams.io/posts/git-commit/) by cbeams. These guidelines are slightly adapted from the ones from [our project](https://git.chalmers.se/courses/dit113/2023/group-12/boombox/-/wikis/Commit%20Template) in DIT113.

Formatting Guidelines
1. Follow English grammar and orthography unless otherwise stated
2. Separate subject from body with a blank line
3. Start the subject line with the Issue ID (e.g. "#1")
4. Limit the subject line to 50 characters
5. Use the imperative case for the subject line
6. Do not end the subject line with a period
7. Wrap the body at 72 characters
8. Use the body to explain what and why vs. how