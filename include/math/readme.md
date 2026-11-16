Convention de la lib Math

camelCase
[[nodiscard]] sur tous ce qui renvoie une valeur sans modifier l'obj
constexpr / const a chaque fois que c'est possible
noexcept partous, on ne doit pas faire d'allocation dans la lib

sur divition par 0 -> pas de verification, c'est a l'user de trancher, mais les fonction ou on peut diviser par 0 peuvent prendre un callback
les verbes agisent sur l'obj, les adj renvoie une copie ex : normalize() modifie l'obj, normalized() renvoie une copie