## Branching
```
git branch → list branches

git branch <branch> → create a new branch

git checkout <branch> → switch to a branch

git checkout -b <branch> → create & switch to a new branch

git merge <branch> → merge another branch into current

git branch -d <branch> → delete a branch locally

git push origin --delete <branch> → delete branch on GitHub
```

## To push code

To push code, please run the following three lines of commands
```
git add <file> → stage a file.

git add . → stage all files.

git commit -m "message" → save staged changes with a message, you can write whatever message you want.
```


## Pulling
```
git fetch → download all branches and updates

git pull → fetch and merge latest changes from remote
```