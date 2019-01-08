# Making Your First Pull Request
Here's the jist:  When you write a new feature, you're going to make a branch that's just for that feature.  You're going to make a branch off `dev`, write some code, commit it to your branch, test the code, make changes, commit it to your branch, etc until you're satisfied with your code.  When you're satisfied with your code, you're going to make a "pull request" against `dev`.  A "pull request" is like saying "hey Kyle, you should accept these changes as your own".  Generally, after you merge your PR, you'll close your branch. Over the season, you'll make a dozen-ish branches.

## Cloning the Repo
Go to https://github.com/Team973/2018-inseason.  See the green box in the upper right that says `Clone or download`?  Yeah that.  Click that.  See the URL in the box? Click the little pastebin icon to the right of it. You should see a popup that says `copied` once you click it.  See it?  Good.

Open terminal on your computer.  Where do you want to keep your repository?  Probably your home dir.  If you want to keep your repository somewhere else, cd to that location.

Type the following command: `git clone <WHATEVER URL WAS IN YOUR CLIPBOARD>`.  Don't actually type the `<WHATEVER URL WAS IN YOUR CLIPBOARD>`.  Put the thing that was in your clipboard there.  You might find that typing contorl v does funky stuff.  If that's the case, try right clicking and clicking paste.  Press enter.  You might need to type in your github username and password.  It should be the same username and password you used to log into github.com.

Type `ls`.  What do you see?  You see whatever was in your directory before plus a new directory called `2018-inseason`.

Type `cd 2018-inseason`. This will change your current directory to 2018-inseason.  This is like in file explorer when you click on a folder and it moves you into that folder.  Good stuff.

Type `ls`.  What do you see?  Probably docs, lib, src, third_party, tools, Dockerfile, install.sh, README.md, StyleGuide.md, WORKSPACE.  I would hope so at least... these are the files in the repo.  Good stuff.

## Make a Branch
There's a handful of ways to make a new branch.  Here's my favorite:

```
git checkout -b <YOUR BRANCH NAME> <WHAT YOU'RE BRANCHING OFF OF>
```

Ohh but there's a few restrictions to keep in mind...
 - Your branch name cannot have spaces in it
 - Your branch name should be descriptive of what you're gonna do with it
 - At 973 we prefer that your branch names use dashes instead of underscores

In general, you're going to be branching off of dev.  This means that when your PR is done, your changes are going to be merged into the dev branch.  Dev is branched off of master.  Every once in a while, we'll take a look at what's in dev and test it a whole bunch.  Once it's been tested we'll announce "this is stable and we are confident in this version" and we'll merge it into master (we don't close dev after we merge it though).

So you ran the command above and it did two things.  The first thing it did was create your branch for you.  The second thing it did was `checkout` that branch.  When you checkout a branch, git changes all the files in your filesystem to match whatever files exist in the branch.  They're probably mostly the same, but they won't be the same in a second.

## Add your Name to the Contributors list
Edit the file `src/Robot.h`.  Do you have a favorite text editor?
 - If Chris L has been within 5 feet of your computer, you probably have VSCode installed so you might as well use that.
 - If you don't have VSCode installed, you can edit the file by typing `nano src/Robot.h`.  nano is a super simple text editor; you can use it to edit any text file.  When you're done making your changes, type control-O followed by enter to save the changes then type control x to exit nano.

At the top of Robot.h there's a list of Contributors.  Add your name to the list.  Every space matters... make sure the `*` lines up with all the other `*`'s and make sure the `-` lines up with all the other `-`.  If it's not exactly right, Kyle is going to reject your PR.

## Compiling
Alright you just changed the source code.  Did you break the world?  Let's check.  This step may take up to 5 minutes the first time you do it.  After you do it the first time it should be almost instantaneous.  Gradle is good stuff.

Type the following to compile the robot code:

    ./gradlew build

If you wanted to download this to the robot and test it there, you would type the following:


    ./gradlew deploy

I don't think we changed enough to merit testing on the robot...

## Commiting
Did the build pass?  Good stuff!  Let's commit our changes.

Just out of curiosity, type `git status`.  What does it say?  I bet it says something like "blah blah blah you have 1 change not staged for commit blah blah blah".  That's git-language for "You edited this file but you haven't added it to a commit yet so I don't know what you're up to".

Good stuff.  Now type `git diff`.  What does it say?  You can see exactly what you changed, every detail down to the last space.  Is this diff what you would expect it to be?  How many lines did you change?  If you just added your name, it should only be one line.  Looking at the output of `git diff` is a good habit to get into because it'll tell you if you accidentally made other changes and it'll remind you what you did.

Are you satisfied with your diff?  You can always go back and make more changes.  Let's commit this.  Type `git add src/Robot.h`.  Now type `git status`.  HARK! THE OUTPUT CHANGED!  Now git says "blah blah blah changes to be committed blah blah blah".  Good stuff.  This is git's way of telling us that next time you type commit, it's going to include these changes in that commit.

Let's commit!  Type `git commit`.  The first time you do this it might ask you to do some configuring on your machine... read the instructions git gives you and do what it says.  It's likely that you'll see a nano session after a second or two.  This is where you write your changelog.  What changes are in this commit?  Describe what you did in traditional-tweet-form (225 characters or less).  When you're done, save like you would any other nano session.  Type control-o then enter to save and type control x to exit nano.

Make sure you read the output of every git command.  What's it saying?  Did it work?  Move on to the next step.  Did it not work?  Read the error and see why git is angry.  If you need help, don't be afraid to ask.

## Push
Type `git push origin <YOUR BRANCH NAME>`

Git will probably ask you to give a username and password.  This is the username and password combo that you use to log into github.com.

This step takes the commit that you made and sends it up to the github server.

## Make a PR
Open your favorite web browser (unless it's IE6) and go to https://github.com/Team973/2018-inseason.  That's a short enough url that you should memorize it eventually.  You will probably see a link that says `New pull request`.  Click that link.  Make sure that the pull request is against `dev` and not `master`.

Write a short summary of what your branch is all about and click the big green button.

## Waiting for Approval
Once you submit your PR, Andrew, Kyle, or Oliver must approve your changes before they are merged.

Andrew, Kyle, or Oliver may request that you make changes before your branch gets merged in.  If that's the case, don't worry.  Edit the file again, make whatever changes you need to make, do the `./gradlew build` thing, do the `git add` thing, do the `git commit` thing, do the `git push` thing, etc.  Once you push, your pull request on github will automatically be updated.

## Merge and Close Branch
Once everything is good and free of errors :fire:, Andrew, Kyle, or Oliver will approve your pull request.  When this happens you get to click the big green `MERGE` button.  If you're done with your branch (you are probably done with your branch), delete it.

Questions? Slack Andrew, Kyle, or Oliver

Want to learn more?  Check out this article for a great explanation of all the steps we did.  https://medium.com/@ashk3l/a-visual-introduction-to-git-9fdca5d3b43a  This article has a lot of helpful diagrams in it - especially for understanding the difference between "staging" and "commiting".
