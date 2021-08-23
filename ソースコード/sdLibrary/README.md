# SDカードライブラリ

## 利用例
```c
#include "sd_lib.hpp"

MySD hoge = MySD();

// ファイルを新規作成して書き込み  
hoge.writeFile(SD, "/hello.txt", "Hello ");  
// 既存ファイルに追記  
mysd.appendFile(SD, "/hello.txt", "World!\n");  
```
etc...
