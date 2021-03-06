from django.conf.urls.defaults import patterns, include, url

# Uncomment the next two lines to enable the admin:
from django.contrib import admin
admin.autodiscover()

urlpatterns = patterns('',
    # Examples:
    # url(r'^$', 'elec4000_web.views.home', name='home'),
    # url(r'^elec4000_web/', include('elec4000_web.foo.urls')),

    # Uncomment the admin/doc line below to enable admin documentation:
    # url(r'^admin/doc/', include('django.contrib.admindocs.urls')),

    # Uncomment the next line to enable the admin:
    url(r'^admin/', include(admin.site.urls)),
	url(r'^get/(?P<node_pk>[0-9]+)', 'ecg.views.get_node'),
	url(r'^get/$', 'ecg.views.index'),
	url(r'^set/(?P<node_pk>[0-9]+)', 'ecg.views.set'),
	url(r'^$', 'ecg.views.index'),
)
