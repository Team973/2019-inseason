# The Official C++ Style Guide of FRC Team 973: The Greybots
This is derived from Google's C++ [Style Guide](https://google.github.io/styleguide/cppguide.html).

Note: We are strictly implementing this style guide on all code written starting on the 2018 Season. Pull requests will be declined once your code does not follow any of the rules defined in this document. :+1:

## Automated tooling
We have automated tooling to help folks format their code correctly.  This tooling depends on clang-format-5.0 or greater.  It is highly recommended that users install the pre-commit hook so that their commits can be checked for style.

The configuration for clang-format is stored in src/.clang-format

### Installing clang-format
The default version of clang-format included with most distributions is too old for our purposes and does not support our style.  You can install a more recent version by running one of the following commands:

#### Ubuntu

These instructions were tested in ubuntu xenial and artful
```
sudo apt install clang-format-5.0
```

#### Mac with Homebrew

These instructions were not tested.  Good luck.
```
brew install clang-format
```

### Installing the pre-commit hook
cd into the WORKSPACE directory and type

```
./tools/style/install_style_check_hook.sh
```

Check that there were no errors in the output.  If all was successful, then next time you commit the script will automatically check your changed files for style violations.  

### Enforcing the style guide in your program
Run this command whenever you want to enforce the style guide in your program.
```
./tools/styles/enforce_style.sh
```

## Style Rules

### Rule #1: Indentions
Instead of using the `tab` character in doing indentions for both the header and source files, use **four spaces**. For parent classes, use 8 spaces.  `Public`, `private`, and ` protected` should not be indented.  
```c++
class Foo
        : public Parent1
        , public Parent2 {
public:
    void function();
private:
    void doSomething();
};
```

### Rule #2: Naming variables according to scope
Member, static, and global variables should be named starting with `m_`, `s_`, or `g_` respectively. In declaring member variables in a class, there should be a space between the type and the name. Declaring constants should have all characters in uppercase and should use underscores instead of spaces.
```c++
int g_value = 10
static const double  TALON_ID = 0;
class MyClass() {
private:
    int m_value;
};
```

### Rule #3: Where to initialize variables
Place a function's variables in the narrowest scope possible, and initialize variables in the declaration. Here is a [reference](https://stackoverflow.com/questions/23345554/the-differences-between-initialize-define-declare-a-variable) for differentiating initialization from declaration
```c++
void function() {
    int var;
    var = 5; //Bad -- initialization separate from declaration
    int num = 10; //Good -- declaration has initialization
    Foo f;  
    for (int i = 0; i < 100; i++) {
        f.DoSomething(i);
    }
}
```

### Rule #4: `#ifndef` and `#pragma` directives
Instead of using the `#ifndef` and `#define` directives, use `#pragma once` in the beginning of header files.
```c++
#pragma once

class MyClass {
public:
    void Foo();
};
```

### Rule #5: Public Inheritance
All inheritance should be public. If you want to do private inheritance, you should be including an instance of the base class as a member instead.
```c++
#pragma once

class ParentClass : public ChildClass {
private:
    int m_value;
};
```

### Rule #6: When to use a Struct vs. Class
Use a struct only for passive objects that carry data; everything else is a class. The struct and class keywords behave almost identically in C++. Please note that structs and classes should have the first letter in their names capitalized.
```c++
struct MyStruct {
    uint32_t time;
    bool state;
};

class MyClass {
public:
    double Divide(double x, double y);
    double Round(double n);
};
```

### Rule #7: Explicit Conversions
The compiler is allowed to make one implicit conversion to resolve the parameters to a function. What this means is that the compiler can use constructors callable with a single parameter to convert from one type to another in order to get the right type for a parameter.
Prefixing the explicit keyword to the constructor prevents the compiler from using that constructor for implicit conversions. Use the explicit keyword for conversion operators and single-argument constructors.
For an in-depth explanation of what the explicit keyword does go [here](https://stackoverflow.com/questions/121162/what-does-the-explicit-keyword-mean).
```c++
class Foo {
    explicit Foo(int x);
};

void Func(Foo f);
```

### Rule #8: No Operator Overloading
C++ permits user code to declare overloaded versions of the built-in operators using the operator keyword, so long as one of the parameters is a user-defined type. The operator keyword also permits user code to define new kinds of literals using operator"", and to define type-conversion functions such as operator bool().

### Rule #9: Declaration Order
Group similar declarations together, placing public parts earlier.
A class definition should usually start with a public: section, followed by protected:, then private:. Omit sections that would be empty.
Within each section, generally prefer grouping similar kinds of declarations together, and generally prefer the following order: types (including typedef, using, and nested structs and classes), constants, factory functions, constructors, assignment operators, destructor, all other methods, data members.
Do not put large method definitions inline in the class definition.

```c++
class Foo {
public:
    void function();
protected:
    static constexpr double CONSTANT = 0;
private:
    void doSomething();
};
```

### Rule #10: Access Control
Make data members private, unless they are static const (and follow the naming convention for constants).

### Rule #11: Control Flow Format
Curly braces around control flow primitives are mandatory even if there's just one line inside. Always put a space between `for`, `if`, and `else`, and `{` in all functions and all control flow statements.
```c++
int total;
for (int i = 0; i < 10; i++) {
    if (total == 100) {
        this.Exit();
    }
    else {
        total += i;
    }
}
```

### Rule #12: Function Overloading
Use overloaded functions (including constructors) only if a reader looking at a call site can get a good idea of what is happening without having to first figure out exactly which overload is being called.
You may write a function that takes a const string& and overload it with another that takes const char*.

```c++
class MyClass {
public:
    void Analyze(const string &text);
    void Analyze(const char *text, size_t textlen);
};
```

### Rule #13: Switch Statements
Use switch statements whenever a sequence of actions/methods must be executed in an orderly manner.
```c++
void Shoot() {
    int count = 1;
    switch(count) {
        case 1:
            StartFlywheel();
            count++;
            break;
        case 2:
            if (UpToSpeed()) {
                count++;
            }
            break;
        case 3:
            Agitate();
            break;
        default:
            break;
    }
};
```

### Rule #14: `#include` directive
When using `#include`, standard libraries should be included in brackets i.e. `#include <stdio.h>`. All other files should use quotes, and should have the full path from the WORKSPACE file i.e. `#include "src/lib/util/Util.h"`.
```c++
#include <stdio.h>
#include "src/RobotInfo.h"

namespace frc {
void printMessage() {
    printf("Hello World!");
}
}
```

### Rule #15: Functions with multiple parameters
Whenever a function has multiple (3 or more) parameters, each line must have at most 3 parameters. Exceeding numbers of parameters must be placed on the next line, and it must be indented and aligned with the first parameter of the previous line.
```c++
double Foo(int a, int b, bool status
           double c) {
    //Do something here       
}
```

### Rule #16: Initializer Lists
If the initializer list spans multiple lines, then there should be one initialization per line.  Each initialization should be indented with 8 spaces. The opening bracket should be on the line of the last initializer. Initializers should be in the order that the member variables appear in the class definition (gcc will warn you if you violate the last rule here).

``` c++
Autonomous::Autonomous(Disabled *disabled)
        : m_noAuto(new NoAuto())
        , m_forwardAuto(new ForwardAuto())
        , m_disabled(disabled) {
    // constructor body
}
```

### Rule #17: `enum` vs. `enum class`
+Use an `enum class` rather than `enum` when possible. if using an `enum`, having the same name for an item in two or more enumerations will give an error and will cause confusion when someone is reading your code. Using an `enum class` instead makes our code easier to read and compile since we have to explicitly call what enum class we are using.

``` c++
enum class PandaType {
    Red,   // if we used regular enum, this would conflict with TulipColor::Red
    Giant,
    Qunling
};

enum class TulipColor {
    Red,    // if we used regular enum, this would conflict with PandaType::Red
    Pink,
    Purple,
    Yellow
};

void SaveThePandas(PandaType type) {
    // TODO
}

int main(void) {
    SaveThePandas(PandaType::Red);
    // You must explicitly reference PandaType to use its own definition of Red. This makes it
    // easier to read because you can look for the definition of PandaType.  
}
```
