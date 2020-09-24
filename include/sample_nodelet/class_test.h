template <class T>
class DummyClass {

    private:
    T in_obj_;
    public:
    /// constructor
    DummyClass(T in_obj) {
        in_obj_ = in_obj;
    }

    ~DummyClass() {}

    void print();

};

