# CEE280 HW08 2026
For Stanford CEE280 - Prof. Gregory Deierlein.
Lectures on optimization by Keith J. Lee.

Begin by either cloning this repository using git, or downloading all files as a .zip folder:
![alt text](readme_assets/repo.png)

 If using the latter, unzip the file. You should now have a folder that matches the contents of this repository.

# Installation instructions

## Julia
First install [Julia](https://julialang.org/install/). This will install the latest version of Julia (v1.13) as well as `juliaup`, a command line tool that will help you update versions in the future.

If you already have Julia installed, run `juliaup update` in a terminal to get v1.13. You can check your version by typing `julia --version`.

![alt text](readme_assets/image.png)

## VS Code
If Julia is the engine, a *text editor* is the steering wheel, pedals, and gear shifter.
Install [Visual Studio Code](https://code.visualstudio.com/), a very popular text editor.

![alt text](readme_assets/image-1.png)

## Setting up VS Code
Open VS Code and install the Julia language extension from the Extensions toolbar on the left side of the window:

![alt text](readme_assets/image-2.png)

You may need to restart VS Code.

Now open this folder using `File > Open Folder...` and select the folder that contains `Project.toml` and the `Problem` files. Open the folder itself, not an individual file: the code looks for its packages in the folder that is open in VS Code.

You're now ready to program using Julia!

# Using Julia
Now navigate to `Problem2.1.jl`, and place your cursor anywhere inside the `begin/end` block at the top of the file and press `shift+enter`. A Julia instance should open up in VS Code and begin running the lines of code inside this block. If you are running this problem for the first time, it will install all the necessary packages for you to start working with the code base. This takes several minutes, and only happens once. Wait until it finishes before moving on.

# Running code
- Keep hitting `shift+enter` to run a block of code and move to the next block. `ctrl+enter` will run a block of code and remain at the current position.
- Try adding your own code, modifying existing code, and running your changes inside the file.

The Julia terminal that appeared when you first started the file is the Julia REPL, akin to Matlab's console. This is the running Julia engine, where you send commands to it by pressing `shift+enter`. You can also work directly inside the REPL, recalling variables you have already defined, or treating it as a very smart calculator.

Try `shift+enter`ing your way through `Problem2.1.jl`. Plots will appear in a separate tab in VS Code.

You need to fill in the missing code, marked with `YOUR CODE HERE`. Until you do, the `isapprox` test line will give an error and the plots will be flat or blank. After completing `k_element` and the `loss` value in `FL`, re-run both functions and then the test line, which should return `true`.
