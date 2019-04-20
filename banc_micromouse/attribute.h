#ifndef ATTRIBUTE_H
#define ATTRIBUTE_H

struct t_attribute
{
    t_attribute(int id) :
    _id(id),
    _value(0)
    {

    }

    virtual void set(int value)
    {
        _value = value;
    }

    int get() const
    {
        return _value;
    }

protected:
    int _id;
    int _value;
};

struct t_attribute_manager
{
    t_attribute_manager()
    {
        for(int index=0; index < _attribute_count_max; ++index)
        {
            _attributes[index] = 0;
        }
    }

    void add(t_attribute * attribute, int id)
    {
        if( id < _attribute_count_max )
        {
            _attributes[id] = attribute;
        }
    }

    void set(int id, int value)
    {
        if( 0 < id && id < _attribute_count_max )
        {
            if( _attributes[id] != 0 )
            {
                _attributes[id]->set(value);
            }
        }
    }

    int get(int id) const
    {
        if( id < _attribute_count_max )
        {
            if( _attributes[id] != 0 )
            {
                return _attributes[id]->get();
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }

private:

    static int const _attribute_count_max = 256;
    t_attribute * _attributes[_attribute_count_max];
};

#endif // ATTRIBUTE_H
