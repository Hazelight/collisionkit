Installation

Following instructions at https://doc.cgal.org/latest/Manual/windows.html#sec-installing-with-vcpkg

Open a terminal to the root of this kit

Get and install vcpkg, a package manager for windows,

```
git clone https://github.com/microsoft/vcpkg
cd vcpkg
bootstrap-vcpkg.bat
```

Then using vcpkg install cgal to get all dependencies for it

```
vcpkg.exe install yasm-tools:x86-windows
vcpkg.exe install cgal:x64-windows
vcpkg.exe install eigen3:x64-windows
```

Installation will take quite some time, around 30 minutes for me

