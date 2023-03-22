find ./BiAITstar -name '*.h' -o -name '*.cpp' | \
xargs clang-format -style=google -i
find ./main.cpp -name '*.h' -o -name '*.cpp' | \
xargs clang-format -style=google -i

