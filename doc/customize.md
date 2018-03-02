## プログラムのカスタマイズ

LittleSLAMは、大きく、スキャンマッチング、センサ融合、ループ閉じ込みという
要素技術から構成されています。
LittleSLAMは、学習用プログラムとして、これらの技術に関して
いくつかのカスタマイズができるように作られています。
下表にカスタマイズのタイプを示します。
それぞれの詳細は、参考書籍[1]を参照してください。


| タイプ              | 内容         |
|:--------------------|:-------------|
| customizeA          | スキャンマッチング基本形  |
| customizeB          | スキャンマッチング改良形1 |
| customizeC          | スキャンマッチング改良形2 |
| customizeD          | スキャンマッチング改良形3 | 
| customizeE          | スキャンマッチング改良形4 |
| customizeF          | スキャンマッチング改良形5 | 
| customizeG          | スキャンマッチング改良形6 | 
| customizeH          | センサ融合による退化の対処 |
| customizeI          | ループ閉じ込み |


カスタマイズのタイプは、SlamLauncher.cppの
関数customizeFrameworkの中で、下記のように指定します。
表のタイプがそのまま関数名であり、指定したい関数を書いて、
それ以外はコメントアウトします。  
デフォルトは、customizeIになっています。

```C++

void SlamLauncher::customizeFramework() {
  fcustom.setSlamFrontEnd(&sfront);
  fcustom.makeFramework();
//  fcustom.customizeG();                   // 使わないのでコメントアウト
  fcustom.customizeI();                     // このカスタマイズを指定

  pcmap = fcustom.getPointCloudMap();
}

```  

また、関数customizeX（X=A to I）は、cui/FrameworkCustomizer.cppで定義されています。  
ユーザが新しいcustomizeXを作って試すことも可能です。


