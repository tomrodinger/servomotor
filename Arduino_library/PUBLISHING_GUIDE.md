# Guide to Publishing the Servomotor Library to Arduino Library Manager

This guide provides step-by-step instructions for publishing the Servomotor library to the Arduino Library Manager, making it available to anyone using the Arduino IDE.

## Prerequisites

- A GitHub account
- Git installed on your computer
- The Servomotor library files prepared with the correct structure

## Step 1: Prepare the Library Structure

The library has already been prepared with the correct structure using the `copy_stuff_to_Arduino.sh` script. This script creates the following structure:

```
Servomotor/
  ├── src/                 # Source files (.cpp)
  ├── examples/            # Example sketches
  │   ├── One_Move/
  │   │   └── One_Move.ino
  │   ├── One_Move_Two_Motors/
  │   │   └── One_Move_Two_Motors.ino
  │   └── Multi_Move/
  │       └── Multi_Move.ino
  ├── keywords.txt         # Keywords for syntax highlighting
  ├── library.properties   # Library metadata
  ├── LICENSE              # MIT License
  └── README.md            # Documentation
```

Run the script to create this structure:

```bash
./copy_stuff_to_Arduino.sh
```

## Step 2: Create a GitHub Repository

1. Go to [GitHub](https://github.com) and sign in to your account.
2. Click the "+" icon in the top-right corner and select "New repository".
3. Name the repository "Arduino-Servomotor" or similar.
4. Add a description: "Arduino library for controlling Gearotons servo motors".
5. Make the repository public.
6. Check "Add a README file" (this will be replaced later).
7. Choose "MIT License" from the "Add a license" dropdown.
8. Click "Create repository".

## Step 3: Clone the Repository

1. On your GitHub repository page, click the "Code" button and copy the repository URL.
2. Open a terminal and navigate to a directory where you want to clone the repository.
3. Run the following command:

```bash
git clone https://github.com/yourusername/Arduino-Servomotor.git
```

## Step 4: Copy the Library Files

1. Run the `copy_stuff_to_Arduino.sh` script to create the properly structured library in your Arduino libraries folder.
2. Copy all files from the Arduino library folder to your cloned repository:

```bash
cp -r /Users/tom/Documents/Arduino/libraries/Servomotor/* /path/to/cloned/Arduino-Servomotor/
```

## Step 5: Commit and Push the Library Files

1. Navigate to your cloned repository:

```bash
cd /path/to/cloned/Arduino-Servomotor
```

2. Add all files to the Git staging area:

```bash
git add .
```

3. Commit the changes:

```bash
git commit -m "Initial library release"
```

4. Push the changes to GitHub:

```bash
git push origin main
```

## Step 6: Create a Release

1. Go to your GitHub repository page.
2. Click on "Releases" in the right sidebar.
3. Click "Create a new release".
4. Enter a tag version (e.g., "v0.9.0") that matches the version in library.properties.
5. Enter a release title (e.g., "Servomotor Library v0.9.0").
6. Add release notes describing the library and its features.
7. Click "Publish release".

## Step 7: Submit to Arduino Library Manager

1. Go to the [Arduino Library Manager GitHub repository](https://github.com/arduino/library-registry).
2. Click on "Fork" in the top-right corner to create your own fork of the repository.
3. In your forked repository, navigate to the "repositories" folder.
4. Click "Add file" > "Create new file".
5. Name the file "servomotor.json".
6. Add the following content to the file:

```json
{
  "name": "Servomotor",
  "keywords": "servo, motor, control, communication",
  "description": "Library for controlling Gearotons servo motors with advanced features like motion control, torque setting, and homing.",
  "repository": {
    "type": "git",
    "url": "https://github.com/yourusername/Arduino-Servomotor.git"
  },
  "authors": {
    "name": "Tom Rodinger",
    "email": "tom.rodinger@alumni.utoronto.ca",
    "url": "https://github.com/tomrodinger"
  },
  "version": "0.9.0",
  "license": "MIT",
  "frameworks": "arduino",
  "platforms": "*"
}
```

7. Click "Commit new file".
8. Click on "Pull requests" > "New pull request".
9. Make sure the base repository is "arduino/library-registry" and the base branch is "main".
10. Click "Create pull request".
11. Add a title and description for your pull request.
12. Click "Create pull request" again.

## Step 8: Wait for Approval

The Arduino team will review your pull request. They may ask for changes or clarifications. Once approved, your library will be available in the Arduino Library Manager.

## Step 9: Update the Library

When you want to update the library:

1. Make your changes to the library files.
2. Update the version number in library.properties.
3. Commit and push the changes to GitHub.
4. Create a new release with a new tag version.

The Arduino Library Manager will automatically detect the new release and make it available to users.

## Additional Resources

- [Arduino Library Specification](https://arduino.github.io/arduino-cli/0.21/library-specification/)
- [Arduino Library Manager FAQ](https://github.com/arduino/Arduino/wiki/Library-Manager-FAQ)