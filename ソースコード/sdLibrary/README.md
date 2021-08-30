# SDカードライブラリ

## 利用例
```c
#include "sd_lib.hpp"

MySD mysd = MySD();

// ファイルを新規作成して書き込み  
mysd.writeFile(SD, "/hello.txt", "Hello ");  
// 既存ファイルに追記  
mysd.appendFile(SD, "/hello.txt", "World!\n");  
// タイムスタンプ付きログ記録
mysd.appendLog("message");
```
etc...
