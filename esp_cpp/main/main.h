#ifndef MAIN_H
#define MAIN_H

class Main final {
private:
    void create_tasks(void);
    [[noreturn]] static void read_and_send(void);
public:
    void start(void);
};

#endif /* MAIN_H */
