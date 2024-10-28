### Welcome to the Contributing Guide

*The purpose of this guide is to ensure that the code can be maintained and understood by everyone.*

## Branches

You should make a new branch to develop a new things and then create a pull request. Pushing commits directly on main is forbidden.

### Branches name

Please use an appropriate keyword like `feature` (or `feat`), `fix`, `refactor`, (`package` *When creating a new package*)

Then indicate a name or keywords separated by `-`

The result should look like this
`feature/Support-xxx-on-rviz`, `fix/Node-xxx`, `package/[Sensor-name]`, ...

## Commit

Please make commits for specific changes and do not make a single commit for the creation of an entire package, for example.

### Commits name

Please use the following format wherever possible.
```
<type>(<scope>): <subject>

<description>

<footer>
```

The `<type>` indicates the nature of the change e.g. `feat`, `fix`, `perf`, `refactor`, `chore`, ...

The `<scope>` indicates where the change is taking place. It can be a package name, a file name, a ros keyword or things like that.

The `<subject>` indicates what we should find in the commit.

The `<description>` indicates further informations about the commit.

The `<footer>` eventually indicates whether this commit contains breaking changes or fixes an issue for instance.

Example
```
feat(Lidar-xx): Adding support for rviz

- Adding a node for supporting rviz
- Create a topic to communicate

BREAKING CHANGE on the package
```