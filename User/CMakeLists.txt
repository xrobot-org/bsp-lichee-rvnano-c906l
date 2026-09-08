# Product application sources; the shared BSP overlay owns the xrobot target.
target_sources(xrobot PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/camera_mailbox.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/platform.cpp"
)
