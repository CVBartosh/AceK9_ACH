#include <Arduino.h>
/// @brief vim_data: the VIM information goes here
struct vim_data {
    int dummy;
};
/// @brief The VIM callback. Your function should follow this signature
typedef void(*vim_on_receive_cb)(const char* data, void* state);
/// @brief Write a string to the VIM serial port
/// @param sz The string to write
void vim_write_sz(const char* sz);
/// @brief Initialize the VIM serial
/// @param callback The callback when a VIM message is received
/// @param state Any associated user state (optional)
void vim_init(vim_on_receive_cb callback,void* state = nullptr);
/// @brief Store VIM state for access from on_receive
/// @param newData the data to store
void vim_store(const vim_data& newData);
/// @brief Load the VIM state
/// @return The stored VIM state
vim_data vim_load();
/// @brief Exchange the old VIM state with a new one
/// @param newData The new VIM state
/// @return The previous VIM state
vim_data vim_exchange(const vim_data& newData);