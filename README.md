# 2021dev
Learning wpilib

### 1. Branch from devTest to create new features or fix bugs

`git checkout devTest`  

`git checkout -b feature/my-feature` OR `git checkout -b bugfix/my-bugfix`

`git add .`

`git commit -m "m"`

`git push -u origin feature/my-feature`

### 2. Merge new feature to devUnstable
  
`git checkout devUnstable`  
`git merge feature/my-feature`

If you have a bug fix for your own branch and don't want to mess with the branch's base code, checkout your branch and branch from it to add your code. After adding the bug fix, merge to your original branch.

### 3. Once fully tested in devUnstable, merge your branch to devTest
### 4. Submit a pull request from devTest to main

Ask someone else to review the code and approve the request. The person who will review your code will test the code on devTest before approving.

[Link to how to create a PR](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) or ask Mr. P
    
## NOTE: Do not branch from main or merge to main!
