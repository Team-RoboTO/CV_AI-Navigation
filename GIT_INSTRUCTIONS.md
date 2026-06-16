# saw all the commits
git log --oneline --graph --decorate --all

to come back, temporary, to a commit committed before the latest one

git switch --detached <code of the commit>

# come to previous commit and remove the latest if not working
git revert HEAD
git push origin main


